#!/usr/bin/env python3
"""Run one read-only semantic plus metric translation preflight."""
from __future__ import annotations

import argparse
import json
import os

from run_nav_soak import SemanticPreflight


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("distance", type=float)
    parser.add_argument("--gateway-url", default=os.environ.get(
        "HWR_GATEWAY_URL", "http://192.168.0.223:8090"))
    parser.add_argument("--vlm-url", default=os.environ.get(
        "LM_STUDIO_URL", "http://127.0.0.1:1234/v1"))
    parser.add_argument("--model", default=os.environ.get(
        "HWR_VLM_MODEL", "qwen/qwen3-vl-4b"))
    parser.add_argument("--margin", type=float, default=0.08)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    token = os.environ.get("HWR_GATEWAY_TOKEN", "")
    if len(token) < 24:
        parser.error("HWR_GATEWAY_TOKEN is required")
    preflight = SemanticPreflight(
        args.gateway_url, token, args.vlm_url, args.model,
        timeout=args.timeout, margin=args.margin, retries=1)
    allowed, report = preflight.assess(args.distance)
    print(json.dumps({"allowed": allowed, **report}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
