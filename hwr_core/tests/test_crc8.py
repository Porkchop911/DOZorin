import random

from hwr_core.rrc.crc8 import CRC8_TABLE, crc8


def crc8_bitwise(data) -> int:
    """Independent oracle: CRC-8/MAXIM, reflected polynomial 0x8C, init 0."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def test_table_matches_bitwise_oracle():
    for i in range(256):
        assert CRC8_TABLE[i] == crc8_bitwise([i]), f"table[{i}]"


def test_known_vectors():
    # canonical CRC-8/MAXIM check value
    assert crc8(b"123456789") == 0xA1
    assert crc8(b"") == 0
    assert crc8(b"\x00") == 0


def test_random_streams_match_oracle():
    rng = random.Random(42)
    for _ in range(200):
        data = bytes(rng.randrange(256) for _ in range(rng.randrange(64)))
        assert crc8(data) == crc8_bitwise(data)
