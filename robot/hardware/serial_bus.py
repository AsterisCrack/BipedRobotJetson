from __future__ import annotations

import logging
import threading

import serial

logger = logging.getLogger(__name__)


class SerialBusError(Exception):
    pass


class SerialBus:
    """
    Thread-safe half-duplex UART bus for SCS/Feetech servos.

    Half-duplex behaviour: TX and RX share the same wire.  Every byte sent by
    the host immediately appears on its own RX line (echo).  We drain that echo
    before reading the servo's response packet.
    """

    def __init__(self, port: str, baud_rate: int, timeout: float = 0.05) -> None:
        self._port = port
        self._baud_rate = baud_rate
        self._timeout = timeout
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()
        self._expect_echo = True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baud_rate,
            timeout=self._timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        logger.info("Serial bus opened on %s @ %d baud", self._port, self._baud_rate)

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
            logger.info("Serial bus closed")

    def __enter__(self) -> "SerialBus":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # I/O (thread-safe)
    # ------------------------------------------------------------------

    def transfer(self, packet: bytes, response_data_len: int) -> bytes:
        """
        Send *packet* and return the servo's response data bytes.

        Args:
            packet: fully-encoded instruction packet (header through checksum)
            response_data_len: number of payload bytes expected in the response
                               (0 for write-only commands that still return a
                               status packet, which has 0 data bytes)

        Returns:
            raw data bytes from the response packet (excludes header/id/len/err/checksum)

        Raises:
            SerialBusError: on timeout, bad header, or checksum mismatch
        """
        if not self.is_open:
            raise SerialBusError("Serial bus is not open")

        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(packet)
            self._serial.flush()

            # Drain TX echo — the host receives its own transmitted bytes
            if self._expect_echo:
                echo = self._serial.read(len(packet))
                if len(echo) == 0:
                    # No echo on this UART; disable echo draining for future transfers.
                    self._expect_echo = False
                elif len(echo) != len(packet):
                    raise SerialBusError(
                        f"Echo drain incomplete: expected {len(packet)} bytes, got {len(echo)}"
                    )

            return self._read_response(response_data_len)

    def send_no_reply(self, packet: bytes) -> None:
        """Send a broadcast packet that expects no response (e.g. SYNC_WRITE)."""
        if not self.is_open:
            raise SerialBusError("Serial bus is not open")

        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(packet)
            self._serial.flush()
            # Drain echo only — no response packet expected
            if self._expect_echo:
                echo = self._serial.read(len(packet))
                if len(echo) == 0:
                    self._expect_echo = False

    def _read_response(self, data_len: int) -> bytes:
        # Response packet: 0xFF 0xFF ID LEN ERR [DATA...] CHECKSUM
        total = 6 + data_len
        raw = self._serial.read(total)

        if len(raw) < total:
            raise SerialBusError(
                f"Response timeout: expected {total} bytes, got {len(raw)}"
            )
        if raw[0] != 0xFF or raw[1] != 0xFF:
            raise SerialBusError(f"Bad response header: {raw[:2].hex()}")

        servo_id = raw[2]
        length = raw[3]   # LEN field = ERR + DATA + CHECKSUM = data_len + 2
        error = raw[4]
        data = raw[5 : 5 + data_len]
        checksum = raw[5 + data_len]

        expected_chk = (~(servo_id + length + error + sum(data))) & 0xFF
        if checksum != expected_chk:
            raise SerialBusError(
                f"Checksum mismatch: got {checksum:#04x}, expected {expected_chk:#04x}"
            )

        if error:
            logger.warning("Servo %d returned error flags: %s", servo_id, bin(error))

        return data
