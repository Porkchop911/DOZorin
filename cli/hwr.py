"""Windows-friendly client for the authenticated HWR gateway."""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import os
import re
import sys
from pathlib import Path
from urllib.parse import quote

DEFAULT_URL = "http://192.168.0.223:8090"
_FLOOR_HAZARD_WORDS = ("wire", "cable", "cord", "clothing", "chair",
                       "obstruction", "clutter", "object on floor")


def _without_explicit_negations(text: str, terms: tuple[str, ...]) -> str:
    for term in sorted(set(terms), key=len, reverse=True):
        text = re.sub(
            rf"\b(?:no|without|free of)\s+(?:visible\s+)?{re.escape(term)}s?\b",
            " ", text)
    return text


def _parse_assessment(raw: str) -> dict:
    raw = raw.strip()
    if raw.startswith("```"):
        raw = raw.split("\n", 1)[-1]
        raw = raw.rsplit("```", 1)[0].strip()
    assessment = json.loads(raw)
    hazard_text = " ".join(str(item).lower()
                           for item in assessment.get("hazards", []))
    corridor = str(assessment.get("visual_corridor", "")).lower()
    corridor_veto_words = (
        "blocked", "clutter", "uncertain", "obstruction", "cable", "wire",
        "cord", "chair", "rack", "leg", "base", "pole", "person")
    checked_hazards = _without_explicit_negations(
        hazard_text, _FLOOR_HAZARD_WORDS)
    checked_corridor = _without_explicit_negations(
        corridor, corridor_veto_words)
    rationale = str(assessment.get("rationale", "")).lower()
    positive_clear = (
        bool(assessment.get("safe_to_translate"))
        and not assessment.get("hazards", [])
        and any(phrase in corridor
                for phrase in ("clear", "open-floor", "open floor", "lane"))
        and not any(word in checked_corridor for word in corridor_veto_words)
        and any(phrase in rationale for phrase in (
            "clearance sufficient", "no visual obstruction",
            "path is clear", "corridor is clear", "no hazards"))
    )
    if assessment.get("recommendation") == "hold" and positive_clear:
        assessment["model_recommendation"] = "hold"
        assessment["recommendation"] = "sensor_preflight"
        assessment["consistency_correction"] = (
            "positive visual evidence contradicted model hold")
    vetoes = []
    if any(word in checked_hazards for word in _FLOOR_HAZARD_WORDS):
        vetoes.append("reported floor hazard")
    if any(word in checked_corridor for word in corridor_veto_words):
        vetoes.append("corridor not visually clear")
    if assessment.get("recommendation") in {"hold", "human_reposition"}:
        vetoes.append("model recommendation requires stopping")
    if assessment.get("safe_to_translate") and vetoes:
        assessment["model_safe_to_translate"] = True
        assessment["safe_to_translate"] = False
        assessment["consistency_veto"] = sorted(set(vetoes))
    return assessment


def _request(args, method: str, path: str, payload=None, binary=False):
    try:
        import requests
    except ImportError as exc:
        raise RuntimeError("install the CLI dependency: python -m pip install requests") from exc
    response = requests.request(method, args.url.rstrip("/") + path,
                                headers={"Authorization": f"Bearer {args.token}"},
                                json=payload, timeout=args.timeout)
    if not response.ok:
        try:
            detail = response.json().get("detail", response.text)
        except ValueError:
            detail = response.text
        raise RuntimeError(f"gateway HTTP {response.status_code}: {detail}")
    return response.content if binary else response.json()


def _run(args) -> int:
    command = args.command
    if command == "status":
        print(json.dumps(_request(args, "GET", "/status"), indent=2))
    elif command == "nav-context":
        print(json.dumps(_request(args, "GET", "/navigation/context"), indent=2))
    elif command == "drive":
        print(json.dumps(_request(args, "POST", "/drive", {
            "vx": args.vx, "vy": args.vy, "wz": args.wz,
            "duration": args.duration}), indent=2))
    elif command == "beep":
        print(json.dumps(_request(args, "POST", "/beep",
                                  {"frequency": args.frequency}), indent=2))
    elif command == "stop":
        print(json.dumps(_request(args, "POST", "/stop"), indent=2))
    elif command == "goto":
        payload = ({"place": args.place} if args.place else
                   {"x": args.x, "y": args.y, "yaw": args.yaw})
        print(json.dumps(_request(args, "POST", "/goto", payload), indent=2))
    elif command == "explore":
        print(json.dumps(_request(args, "POST", f"/explore/{args.mode}"), indent=2))
    elif command == "arm":
        if args.arm_command == "pose":
            path, payload = "/arm/pose", {"name": args.name,
                                           "duration_ms": args.duration_ms}
        elif args.arm_command == "reach":
            path, payload = "/arm/reach", {
                "x": args.x, "y": args.y, "z": args.z,
                "tool_pitch": args.tool_pitch, "duration_ms": args.duration_ms}
        else:
            path, payload = "/arm/aim", {
                "bearing": args.bearing, "elevation": args.elevation,
                "duration_ms": args.duration_ms}
        print(json.dumps(_request(args, "POST", path, payload), indent=2))
    elif command == "gripper":
        print(json.dumps(_request(args, "POST", "/gripper", {
            "command": args.mode, "duration_ms": args.duration_ms}), indent=2))
    elif command in ("snapshot", "map"):
        endpoint = "/snapshot" if command == "snapshot" else "/map.png"
        Path(args.output).write_bytes(_request(args, "GET", endpoint, binary=True))
        print(args.output)
    elif command == "describe":
        import requests
        image = _request(args, "GET", "/snapshot", binary=True)
        payload = {"model": args.model, "temperature": 0.2, "messages": [{
            "role": "user", "content": [
                {"type": "text", "text": args.question},
                {"type": "image_url", "image_url": {"url":
                    "data:image/jpeg;base64," + base64.b64encode(image).decode("ascii")}},
            ]}]}
        response = requests.post(args.lm_studio_url.rstrip("/") + "/chat/completions",
                                 json=payload, timeout=args.vlm_timeout)
        response.raise_for_status()
        print(response.json()["choices"][0]["message"]["content"])
    elif command == "assess":
        import requests
        context = _request(args, "GET", "/navigation/context")
        image = _request(args, "GET", "/snapshot", binary=True)
        prompt = (
            "You are an advisory vision layer for an attended indoor robot. "
            "Metric sensor context is authoritative; never overrule a blocked "
            "clearance. Do not identify people. A doorway is visible only when "
            "an open passage with floor continuing into another space is clearly "
            "seen; windows, curtains, blank walls, closed doors, and furniture "
            "gaps are not doorways. Treat windows, glazed doors, mirrors, and "
            "reflective planes as optical boundaries rather than traversable "
            "openings. A person is visible only when a human body "
            "is clearly present. Return one compact JSON object "
            "with keys: safe_to_translate (boolean), doorway_visible (boolean), "
            "person_visible (boolean), hazards (array of short strings), "
            "visual_corridor (string), and recommendation (one of hold, "
            "human_reposition, rotate_only, sensor_preflight), doorway_evidence "
            "(short string or empty), and person_evidence (short string or empty). "
            "Use sensor_preflight when the visible near-field route appears safe "
            "but metric checks are still required. Use hold only for an observed "
            "or uncertain visual concern; not commanding motion is not a reason "
            "to hold. "
            f"Sensor context: {json.dumps(context, separators=(',', ':'))}. "
            f"Operator question: {args.question}")
        payload = {"model": args.model, "temperature": 0.1, "messages": [{
            "role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url":
                    "data:image/jpeg;base64," + base64.b64encode(image).decode("ascii")}},
            ]}]}
        response = requests.post(args.lm_studio_url.rstrip("/") + "/chat/completions",
                                 json=payload, timeout=args.vlm_timeout)
        response.raise_for_status()
        raw = response.json()["choices"][0]["message"]["content"]
        print(json.dumps(_parse_assessment(raw), indent=2))
    elif command == "watch":
        asyncio.run(_watch(args))
    return 0


async def _watch(args) -> None:
    try:
        import websockets
    except ImportError as exc:
        raise RuntimeError("install watch support: python -m pip install websockets") from exc
    base = args.url.rstrip("/")
    ws_url = ("wss://" + base[8:] if base.startswith("https://") else
              "ws://" + base[7:] if base.startswith("http://") else base)
    async with websockets.connect(f"{ws_url}/telemetry?token={quote(args.token)}") as socket:
        async for message in socket:
            print(json.dumps(json.loads(message), indent=2), flush=True)


def parser() -> argparse.ArgumentParser:
    out = argparse.ArgumentParser(prog="hwr")
    out.add_argument("--url", default=os.environ.get("HWR_GATEWAY_URL", DEFAULT_URL))
    out.add_argument("--token", default=os.environ.get("HWR_GATEWAY_TOKEN", ""))
    out.add_argument("--timeout", type=float, default=5.0)
    commands = out.add_subparsers(dest="command", required=True)
    commands.add_parser("status")
    commands.add_parser("nav-context")
    commands.add_parser("watch")
    drive = commands.add_parser("drive")
    drive.add_argument("vx", type=float)
    drive.add_argument("vy", type=float)
    drive.add_argument("wz", type=float)
    drive.add_argument("--duration", type=float, default=0.25)
    beep = commands.add_parser("beep")
    beep.add_argument("frequency", nargs="?", type=int, default=1500)
    commands.add_parser("stop")
    goto = commands.add_parser("goto")
    goto.add_argument("place", nargs="?")
    goto.add_argument("--x", type=float)
    goto.add_argument("--y", type=float)
    goto.add_argument("--yaw", type=float, default=0.0)
    explore = commands.add_parser("explore")
    explore.add_argument("mode", choices=("start", "stop"))
    arm = commands.add_parser("arm")
    arm_commands = arm.add_subparsers(dest="arm_command", required=True)
    pose = arm_commands.add_parser("pose")
    pose.add_argument("name")
    pose.add_argument("--duration-ms", type=int, default=1500)
    reach = arm_commands.add_parser("reach")
    reach.add_argument("x", type=float); reach.add_argument("y", type=float)
    reach.add_argument("z", type=float); reach.add_argument("--tool-pitch", type=float, default=0.0)
    reach.add_argument("--duration-ms", type=int, default=1500)
    aim = arm_commands.add_parser("aim")
    aim.add_argument("bearing", type=float); aim.add_argument("--elevation", type=float, default=0.0)
    aim.add_argument("--duration-ms", type=int, default=1000)
    gripper = commands.add_parser("gripper")
    gripper.add_argument("mode", choices=("open", "close"))
    gripper.add_argument("--duration-ms", type=int, default=800)
    snapshot = commands.add_parser("snapshot")
    snapshot.add_argument("output", nargs="?", default="snapshot.jpg")
    map_command = commands.add_parser("map")
    map_command.add_argument("output", nargs="?", default="map.png")
    describe = commands.add_parser("describe")
    describe.add_argument("question", nargs="?", default=(
        "Briefly describe the room, traversable floor, obstacles, and visible people. "
        "Do not identify anyone."))
    describe.add_argument("--lm-studio-url",
                          default=os.environ.get("LM_STUDIO_URL", "http://127.0.0.1:1234/v1"))
    describe.add_argument("--model",
                          default=os.environ.get("HWR_VLM_MODEL", "qwen/qwen3-vl-4b"))
    describe.add_argument("--vlm-timeout", type=float, default=120.0)
    assess = commands.add_parser("assess")
    assess.add_argument("question", nargs="?", default=(
        "Judge whether the robot should remain stopped or may proceed to a "
        "separate metric sensor preflight."))
    assess.add_argument("--lm-studio-url",
                        default=os.environ.get("LM_STUDIO_URL", "http://127.0.0.1:1234/v1"))
    assess.add_argument("--model",
                        default=os.environ.get("HWR_VLM_MODEL", "qwen/qwen3-vl-4b"))
    assess.add_argument("--vlm-timeout", type=float, default=120.0)
    return out


def main() -> int:
    args = parser().parse_args()
    if not args.token:
        parser().error("set HWR_GATEWAY_TOKEN or pass --token")
    if args.command == "goto" and not args.place and (args.x is None or args.y is None):
        parser().error("goto requires a place or both --x and --y")
    try:
        return _run(args)
    except (RuntimeError, OSError) as exc:
        print(f"hwr: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
