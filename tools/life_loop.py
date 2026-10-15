"""Soudruhu's idle life: wake the Hermes robot profile every few minutes in one persistent session.

Runs on the PC:  python tools/life_loop.py [--interval-min 5] [--ticks 0]
Ctrl+C stops it. Each tick is logged to evidence/phase10/life_loop.log.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# Existing folder (no new dirs without asking): the agent phase evidence.
STATE = ROOT / "evidence" / "phase10"
SESSION_FILE = STATE / "life_session.txt"
PROMPT = (
    "[life tick {n}, {time}] Nobody is giving you a task right now; live your life, comrade. "
    "Check your surroundings (seen_objects, and describe_view if something is new), then roam: "
    "wander to a new spot (max_distance 1.5) and look around. Remember anything notable "
    "(people, objects, changes) in memory. When you find something new, say one short "
    "in-character remark aloud about it (say tool), at most once per tick. If a command fails with 'estop latched', reset_estop. "
    "If a move fails on localization, call relocalize, then retry once. "
    "Finish within a few actions and reply with one line: what you did and saw."
)


def session_id(output: str) -> str | None:
    match = re.search(r"session_id:\s*(\S+)", output)
    return match.group(1) if match else None


def tick(n: int, timeout: float) -> str:
    cmd = ["hermes", "-p", "robot", "chat", "-Q", "-q",
           PROMPT.format(n=n, time=datetime.now().strftime("%H:%M"))]
    if SESSION_FILE.exists():
        cmd[4:4] = ["--resume", SESSION_FILE.read_text().strip()]
    result = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                            errors="replace", timeout=timeout)
    output = result.stdout + result.stderr
    if not SESSION_FILE.exists() and (sid := session_id(output)):
        SESSION_FILE.write_text(sid)
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interval-min", type=float, default=5.0)
    parser.add_argument("--ticks", type=int, default=0, help="0 = run until Ctrl+C")
    parser.add_argument("--tick-timeout-min", type=float, default=12.0)
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # Windows console is cp1252
    n = 0
    while args.ticks == 0 or n < args.ticks:
        n += 1
        started = time.monotonic()
        try:
            output = tick(n, args.tick_timeout_min * 60)
        except subprocess.TimeoutExpired:
            output = "tick timed out"
        last = [line for line in output.splitlines()
                if line.strip() and "Unknown toolsets" not in line
                and not line.startswith(("session_id:", "↻"))][-1:] or ["(no output)"]
        line = f"{datetime.now():%F %T} tick {n}: {last[0][:300]}"
        print(line, flush=True)
        with open(STATE / "life_loop.log", "a", encoding="utf-8") as log:
            log.write(line + "\n")
        time.sleep(max(0.0, args.interval_min * 60 - (time.monotonic() - started)))


if __name__ == "__main__":
    assert session_id("x\nsession_id: 20261014_130000_abc\nok") == "20261014_130000_abc"
    assert session_id("nothing") is None
    main()
