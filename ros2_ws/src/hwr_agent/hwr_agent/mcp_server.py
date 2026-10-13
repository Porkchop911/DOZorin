"""Optional MCP exposure of the same bounded gateway skills."""
from __future__ import annotations

import functools
import json
import os
from pathlib import Path

from .skills import GatewaySkills

# Runs from the source tree on the PC (Hermes); tools.json is the single manifest.
MANIFEST = Path(__file__).resolve().parents[1] / "config" / "tools.json"


def _refusals_as_results(fn):
    # Hermes trips its MCP circuit breaker after 3 tool errors; a robot refusal is an answer, not an outage.
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except (RuntimeError, ValueError) as exc:
            return {"refused": True, "reason": str(exc)}
    return wrapper


def main():
    try:
        from mcp.server.fastmcp import FastMCP
    except ImportError as exc:
        raise SystemExit("install MCP support: python3 -m pip install mcp") from exc
    skills = GatewaySkills(dry_run=os.environ.get("HWR_DRY_RUN") == "1",
                           max_actions=int(os.environ.get("HWR_MAX_ACTIONS", "20")))
    server = FastMCP("hwr")
    for item in json.loads(MANIFEST.read_text(encoding="utf-8")):
        name = item["function"]["name"]
        server.tool(name=name, description=item["function"]["description"])(
            _refusals_as_results(getattr(skills, name)))
    server.run()


if __name__ == "__main__":
    main()
