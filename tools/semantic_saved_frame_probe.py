#!/usr/bin/env python3
"""Run the production semantic preflight against a saved camera frame.

The probe hosts a loopback-only, battery-free navigation context and serves the
selected JPEG as `/snapshot`.  It never contacts or commands the robot.
"""
from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread

from hwr_core.navigation import SemanticPreflightClient


def _context() -> dict:
    return {
        "pose": [0.0, 0.0, 0.0],
        "localization": {"healthy": True, "fresh": True, "state": "offline_probe"},
        "current_goal": None,
        "drive_active": False,
        "camera_fresh": True,
        "camera_forward": True,
        "hard_clearance_m": {"forward": None},
        "trapped_start": False,
        "clearance_limit_m": 1.5,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--distance", type=float, default=0.10)
    parser.add_argument("--margin", type=float, default=0.25)
    parser.add_argument("--vlm-url", default="http://127.0.0.1:1234/v1")
    parser.add_argument("--model", default="qwen/qwen3-vl-4b")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--expect", choices=("boundary", "clear"))
    args = parser.parse_args()
    image = args.image.read_bytes()
    context = _context()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - stdlib handler API
            if self.path == "/navigation/context":
                body = json.dumps(context).encode("utf-8")
                content_type = "application/json"
            elif self.path == "/snapshot":
                body = image
                content_type = "image/jpeg"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args) -> None:
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        client = SemanticPreflightClient(
            f"http://127.0.0.1:{server.server_port}",
            "offline-probe-token-00000000",
            args.vlm_url,
            args.model,
            timeout=args.timeout,
            margin=args.margin,
            retries=1,
        )
        allowed, report = client.assess(args.distance)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)

    assessment = report["assessment"]
    result = {
        "image": str(args.image),
        "allowed": allowed,
        "glass_boundary_visible": assessment.get("glass_boundary_visible"),
        "glass_boundary_in_corridor": assessment.get("glass_boundary_in_corridor"),
        "glass_boundary_evidence": assessment.get("glass_boundary_evidence"),
        "recommendation": assessment.get("recommendation"),
        "rationale": assessment.get("rationale"),
        "vlm_attempts": report.get("vlm_attempts"),
        "reasons": report.get("reasons"),
    }
    print(json.dumps(result, indent=2))
    if args.expect == "boundary":
        return 0 if (
            not allowed
            and assessment.get("glass_boundary_visible") is True
            and assessment.get("glass_boundary_in_corridor") is True
        ) else 2
    if args.expect == "clear":
        return 0 if (
            allowed
            and assessment.get("glass_boundary_in_corridor") is False
        ) else 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
