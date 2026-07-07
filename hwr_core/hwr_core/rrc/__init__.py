"""Clean-room implementation of the RRC serial protocol (STM32 controller).

Spec: ``plan/90-clean-room-reference.md``. Layers:
``crc8``/``frames`` (wire format) -> ``codecs`` (payloads) ->
``transport`` (serial + RX thread) -> ``board`` (the API you want).
"""
from . import codecs
from .board import Board
from .codecs import ImuSample
from .crc8 import crc8
from .frames import Decoder, Frame, Function, encode
from .transport import Transport, open_rrc

__all__ = [
    "Board", "Decoder", "Frame", "Function", "ImuSample",
    "Transport", "codecs", "crc8", "encode", "open_rrc",
]
