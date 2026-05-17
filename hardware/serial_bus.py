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

    def __init__(self, port: str, baud_rate: int, timeout: float = 0.05, expect_echo: bool = False) -> None:
        self._port = port
        self._baud_rate = baud_rate
        self._timeout = timeout
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()
        self._expect_echo = expect_echo

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
                    logger.warning(
                        "Echo drain returned 0 bytes on %s; skipping this drain",
                        self._port,
                    )
                elif len(echo) != len(packet):
                    raise SerialBusError(
                        f"Echo drain incomplete: expected {len(packet)} bytes, got {len(echo)}"
                    )

            return self._read_response(response_data_len)

    def sync_read(
        self, packet: bytes, servo_ids: list[int], data_len: int
    ) -> dict[int, bytes]:
        """
        Send a SYNC_READ broadcast and collect one response per servo.

        Holds the bus lock for the entire transaction so no other transfer
        can interleave between the request and the N responses.

        Args:
            packet:    fully-encoded SYNC_READ instruction packet
            servo_ids: ordered list of servo IDs expected to respond
            data_len:  payload bytes expected in each response

        Returns:
            {servo_id: data_bytes} for every servo that replied successfully.
            Servos that time out or return bad checksums are silently omitted.
        """
        if not self.is_open:
            raise SerialBusError("Serial bus is not open")

        stride = 6 + data_len              # bytes per servo response
        total_rx = stride * len(servo_ids)

        results: dict[int, bytes] = {}
        with self._lock:
            self._serial.write(packet)
            # No flush() — the hardware UART starts DMA immediately after write().
            # flush() (tcdrain) on the Jetson L4T kernel waits one full kernel tick
            # (~10 ms) regardless of packet size, adding 10 ms of dead time per cycle.
            # The servo responses arrive during that wait and pile up in the RX buffer;
            # skipping flush() and going straight to read() cuts ~10 ms per cycle.
            if self._expect_echo:
                echo = self._serial.read(len(packet))
                if len(echo) == 0:
                    logger.warning(
                        "SYNC_READ echo drain returned 0 bytes on %s; skipping this drain",
                        self._port,
                    )
                elif len(echo) != len(packet):
                    raise SerialBusError(
                        f"SYNC_READ echo drain incomplete: "
                        f"expected {len(packet)} bytes, got {len(echo)}"
                    )

            # One bulk read instead of N individual reads — eliminates N-1 syscalls.
            bulk = self._serial.read(total_rx)
            if len(bulk) < total_rx:
                logger.debug(
                    "SYNC_READ bulk: short read — expected %d bytes, got %d; "
                    "clearing input buffer",
                    total_rx, len(bulk),
                )
                self._serial.reset_input_buffer()
                return results

            for i, sid in enumerate(servo_ids):
                chunk = bulk[i * stride : (i + 1) * stride]
                try:
                    results[sid] = self._parse_chunk(chunk, data_len)
                except SerialBusError as exc:
                    logger.debug("SYNC_READ: bad response from servo %d: %s", sid, exc)

        return results

    def send_no_reply(self, packet: bytes) -> None:
        """Send a broadcast packet that expects no response (e.g. SYNC_WRITE)."""
        if not self.is_open:
            raise SerialBusError("Serial bus is not open")

        with self._lock:
            self._serial.write(packet)
            # No flush() — bytes transmit via DMA while the bus manager sleeps.
            # Drain echo only — no response packet expected
            if self._expect_echo:
                echo = self._serial.read(len(packet))
                if len(echo) == 0:
                    logger.warning(
                        "SYNC_WRITE echo drain returned 0 bytes on %s; skipping this drain",
                        self._port,
                    )

    def _read_response(self, data_len: int) -> bytes:
        # Response packet: 0xFF 0xFF ID LEN ERR [DATA...] CHECKSUM
        total = 6 + data_len
        raw = self._serial.read(total)

        if len(raw) < total:
            raise SerialBusError(
                f"Response timeout: expected {total} bytes, got {len(raw)}"
            )
        return self._parse_chunk(raw, data_len)

    def _parse_chunk(self, chunk: bytes, data_len: int) -> bytes:
        """Validate and extract data from one servo response chunk (no I/O)."""
        if chunk[0] != 0xFF or chunk[1] != 0xFF:
            raise SerialBusError(f"Bad response header: {chunk[:2].hex()}")

        servo_id = chunk[2]
        length   = chunk[3]   # ERR + DATA + CHECKSUM = data_len + 2
        error    = chunk[4]
        data     = chunk[5 : 5 + data_len]
        checksum = chunk[5 + data_len]

        expected_chk = (~(servo_id + length + error + sum(data))) & 0xFF
        if checksum != expected_chk:
            raise SerialBusError(
                f"Checksum mismatch: got {checksum:#04x}, expected {expected_chk:#04x}"
            )

        if error:
            logger.warning("Servo %d returned error flags: %s", servo_id, bin(error))

        return data
