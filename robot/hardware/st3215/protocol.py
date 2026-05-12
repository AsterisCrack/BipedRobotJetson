"""
SCS/Feetech packet encoding and decoding.

Packet format  (instruction):
    0xFF  0xFF  ID  LEN  INSTR  [PARAMS...]  CHECKSUM
    LEN = len(PARAMS) + 2   (INSTR byte + CHECKSUM byte)
    CHECKSUM = ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF

Response packet (status):
    0xFF  0xFF  ID  LEN  ERROR  [DATA...]  CHECKSUM
    LEN = len(DATA) + 2

All multi-byte values are little-endian (LSB first).
"""
from __future__ import annotations

from robot.hardware.st3215.registers import Instr


class ProtocolError(Exception):
    pass


# ---------------------------------------------------------------------------
# Checksum
# ---------------------------------------------------------------------------

def _checksum(id_: int, length: int, instr_or_err: int, params: bytes) -> int:
    return (~(id_ + length + instr_or_err + sum(params))) & 0xFF


# ---------------------------------------------------------------------------
# Instruction packet builders
# ---------------------------------------------------------------------------

def encode_packet(id_: int, instr: int, params: bytes = b"") -> bytes:
    length = len(params) + 2
    chk = _checksum(id_, length, instr, params)
    return bytes([0xFF, 0xFF, id_, length, instr]) + params + bytes([chk])


def encode_ping(id_: int) -> bytes:
    return encode_packet(id_, Instr.PING)


def encode_read(id_: int, register: int, length: int) -> bytes:
    return encode_packet(id_, Instr.READ, bytes([register, length]))


def encode_write(id_: int, register: int, data: bytes) -> bytes:
    return encode_packet(id_, Instr.WRITE, bytes([register]) + data)


def encode_reg_write(id_: int, register: int, data: bytes) -> bytes:
    """Buffered write — executes on ACTION broadcast."""
    return encode_packet(id_, Instr.REG_WRITE, bytes([register]) + data)


def encode_action(id_: int = 0xFE) -> bytes:
    """Broadcast ACTION to execute all pending REG_WRITEs."""
    return encode_packet(id_, Instr.ACTION)


def encode_sync_read(register: int, data_len: int, servo_ids: list[int]) -> bytes:
    """
    Build a SYNC_READ broadcast packet.

    Each listed servo will respond in turn with data_len bytes starting at register.

    Args:
        register:  starting register address
        data_len:  bytes to read per servo
        servo_ids: list of servo IDs to query
    """
    params = bytes([register, data_len] + servo_ids)
    return encode_packet(0xFE, Instr.SYNC_READ, params)


def encode_sync_write(
    register: int,
    data_len: int,
    servo_data: list[tuple[int, bytes]],
) -> bytes:
    """
    Build a SYNC_WRITE broadcast packet.

    Args:
        register:   starting register address
        data_len:   bytes per servo
        servo_data: list of (servo_id, data_bytes) tuples
    """
    params = bytes([register, data_len])
    for servo_id, data in servo_data:
        if len(data) != data_len:
            raise ProtocolError(
                f"SYNC_WRITE data_len mismatch for servo {servo_id}: "
                f"expected {data_len}, got {len(data)}"
            )
        params += bytes([servo_id]) + data
    return encode_packet(0xFE, Instr.SYNC_WRITE, params)


# ---------------------------------------------------------------------------
# Multi-byte value encoding
# ---------------------------------------------------------------------------

def pack_u16(value: int) -> bytes:
    """Pack unsigned 16-bit integer, little-endian."""
    value = max(0, min(0xFFFF, value))
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


def unpack_u16(data: bytes, offset: int = 0) -> int:
    """Unpack unsigned 16-bit integer from little-endian bytes."""
    return data[offset] | (data[offset + 1] << 8)


def pack_s16(value: int) -> bytes:
    """Pack signed 16-bit integer, little-endian (two's complement)."""
    value = max(-32768, min(32767, value))
    if value < 0:
        value += 65536
    return pack_u16(value)


def unpack_s16(data: bytes, offset: int = 0) -> int:
    raw = unpack_u16(data, offset)
    return raw if raw < 32768 else raw - 65536


def steps_to_bytes(steps: int) -> bytes:
    """Encode a 12-bit position (0-4095) as 2 little-endian bytes."""
    steps = max(0, min(4095, steps))
    return pack_u16(steps)


def bytes_to_steps(data: bytes, offset: int = 0) -> int:
    return unpack_u16(data, offset) & 0x0FFF
