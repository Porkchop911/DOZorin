"""Minimal OpenAI-compatible tool loop for LM Studio."""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import requests
from ament_index_python.packages import get_package_share_directory

from .skills import GatewaySkills


def run_mission(text: str, dry_run: bool = False) -> str:
    share = Path(get_package_share_directory("hwr_agent"))
    tools = json.loads((share / "config/tools.json").read_text(encoding="utf-8"))
    skills = GatewaySkills(dry_run=dry_run)
    messages = [{"role": "system", "content": (
        "You command a forward-only indoor robot. Use only provided tools. "
        "Never claim an action or observation that a tool did not confirm. "
        "Stop when a safety guard rejects an action. Power decisions and power "
        "telemetry belong exclusively to the human operator. Semantic scene "
        "assessment is advisory; metric navigation guards remain authoritative.")},
        {"role": "user", "content": text}]
    endpoint = os.environ.get("LM_STUDIO_URL", "http://127.0.0.1:1234/v1").rstrip("/")
    model = os.environ.get("HWR_AGENT_MODEL", "qwen3.5-9b")
    transcript = Path.home() / "hwr/state/missions" / f"mission_{int(time.time())}.jsonl"
    transcript.parent.mkdir(parents=True, exist_ok=True)
    for _ in range(12):
        response = requests.post(endpoint + "/chat/completions", json={
            "model": model, "temperature": 0.3, "messages": messages, "tools": tools},
            timeout=120.0)
        response.raise_for_status()
        message = response.json()["choices"][0]["message"]
        messages.append(message)
        with transcript.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(message) + "\n")
        calls = message.get("tool_calls") or []
        if not calls:
            return message.get("content", "")
        for call in calls:
            name = call["function"]["name"]
            arguments = json.loads(call["function"].get("arguments") or "{}")
            try:
                output = skills.call(name, arguments)
            except Exception as exc:
                output = {"error": str(exc)}
            tool_message = {"role": "tool", "tool_call_id": call["id"],
                            "content": json.dumps(output)}
            messages.append(tool_message)
            with transcript.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(tool_message) + "\n")
    raise RuntimeError("mission exceeded tool-loop turn limit")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mission")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    print(run_mission(args.mission, args.dry_run))
