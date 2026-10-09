"""PGM/YAML occupancy map loading compatible with ROS map files."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml


def load_map(yaml_path: str | Path) -> tuple[np.ndarray, float, tuple[float, float]]:
    path = Path(yaml_path)
    metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw = (path.parent / metadata["image"]).read_bytes()
    tokens, index = [], 0
    while len(tokens) < 4:
        while index < len(raw) and raw[index] in b" \t\r\n":
            index += 1
        if raw[index:index + 1] == b"#":
            index = raw.index(b"\n", index) + 1
            continue
        end = index
        while end < len(raw) and raw[end] not in b" \t\r\n":
            end += 1
        tokens.append(raw[index:end])
        index = end
    if tokens[0] != b"P5" or int(tokens[3]) != 255:
        raise ValueError("only binary 8-bit PGM maps are supported")
    width, height = int(tokens[1]), int(tokens[2])
    index += 1  # exactly one header delimiter before the binary raster
    pixels = np.frombuffer(raw[index:index + width * height], dtype=np.uint8)
    if pixels.size != width * height:
        raise ValueError("truncated PGM raster")
    pixels = np.flipud(pixels.reshape(height, width))
    occupancy = np.full(pixels.shape, -1, dtype=np.int8)
    occupancy[pixels >= 250] = 0
    occupancy[pixels <= 50] = 100
    origin = metadata["origin"]
    return occupancy, float(metadata["resolution"]), (float(origin[0]), float(origin[1]))
