import random

import pytest

from hwr_core.rrc.crc8 import crc8
from hwr_core.rrc.frames import Decoder, Frame, Function, encode


def test_encode_layout():
    frame = encode(Function.MOTOR, b"\x01\x02")
    assert frame[:2] == b"\xaa\x55"
    assert frame[2] == 3          # function
    assert frame[3] == 2          # length
    assert frame[4:6] == b"\x01\x02"
    assert frame[6] == crc8(frame[2:6])


def test_encode_empty_payload():
    frame = encode(Function.IMU)
    assert len(frame) == 5
    assert frame[3] == 0


def test_encode_rejects_bad_input():
    with pytest.raises(ValueError):
        encode(11, b"")
    with pytest.raises(ValueError):
        encode(-1, b"")
    with pytest.raises(ValueError):
        encode(0, bytes(256))


def roundtrip(frames_bytes, chunk_size=None):
    dec = Decoder()
    out = []
    if chunk_size is None:
        out += dec.feed(frames_bytes)
    else:
        for i in range(0, len(frames_bytes), chunk_size):
            out += dec.feed(frames_bytes[i:i + chunk_size])
    return dec, out


def test_roundtrip_all_chunkings():
    stream = b"".join(encode(f, bytes([f] * f)) for f in range(11))
    for chunk in (None, 1, 2, 3, 7):
        _, frames = roundtrip(stream, chunk)
        assert [(fr.function, fr.data) for fr in frames] == \
               [(f, bytes([f] * f)) for f in range(11)]


def test_resync_after_garbage_prefix():
    stream = b"\x00\xff\xaa\x12\x55" + encode(Function.SYS, b"\x04\x10\x27")
    _, frames = roundtrip(stream, 1)
    assert frames == [Frame(Function.SYS, b"\x04\x10\x27")]


def test_resync_after_crc_error():
    bad = bytearray(encode(Function.IMU, bytes(24)))
    bad[-1] ^= 0xFF
    good = encode(Function.SYS, b"\x04\x00\x30")
    dec, frames = roundtrip(bytes(bad) + good, 1)
    assert frames == [Frame(Function.SYS, b"\x04\x00\x30")]
    assert dec.crc_errors == 1


def test_rejects_unknown_function_then_recovers():
    stream = b"\xaa\x55\x0b\x00\x00" + encode(Function.KEY, b"\x01\x20")
    dec, frames = roundtrip(stream)
    assert frames == [Frame(Function.KEY, b"\x01\x20")]
    assert dec.resyncs >= 1


def test_header_bytes_inside_payload():
    payload = b"\xaa\x55\xaa\x55"
    _, frames = roundtrip(encode(Function.GAMEPAD, payload), 1)
    assert frames == [Frame(Function.GAMEPAD, payload)]


def test_double_aa_header_still_syncs():
    stream = b"\xaa" + encode(Function.KEY, b"\x02\x01")
    _, frames = roundtrip(stream, 1)
    assert frames == [Frame(Function.KEY, b"\x02\x01")]


def test_truncated_frame_then_stream_recovers():
    truncated = encode(Function.IMU, bytes(24))[:10]  # cut mid-data
    # decoder swallows following bytes as the missing payload, fails CRC,
    # then resyncs somewhere inside the ongoing stream of good frames
    good = encode(Function.SYS, b"\x04\xaa\x2e")
    _, frames = roundtrip(truncated + good * 6, 1)
    assert frames.count(Frame(Function.SYS, b"\x04\xaa\x2e")) >= 3


def test_fuzz_noise_never_crashes():
    rng = random.Random(7)
    dec = Decoder()
    dec.feed(bytes(rng.randrange(256) for _ in range(20000)))
    # whatever it decoded, it must still work afterwards
    frames = dec.feed(encode(Function.SYS, b"\x04\x01\x02"))
    assert frames[-1] == Frame(Function.SYS, b"\x04\x01\x02")
