"""Run one bounded, evidence-producing autonomous exploration sector.

This monitor deliberately uses only battery-free gateway endpoints.  It tails the
exploration journal, captures a camera image and navigation context after every
successful translation, and always stops exploration and motion on
exit.
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def _token() -> str:
    token = os.environ.get("HWR_GATEWAY_TOKEN", "").strip()
    if token:
        return token
    token_path = Path.home() / ".config" / "hwr" / "token"
    if token_path.exists():
        return token_path.read_text(encoding="utf-8").strip()
    raise RuntimeError("HWR_GATEWAY_TOKEN is not configured")


def _request(base_url: str, token: str, method: str, path: str) -> bytes:
    request = Request(
        base_url.rstrip("/") + path,
        method=method,
        headers={"Authorization": f"Bearer {token}"},
    )
    try:
        with urlopen(request, timeout=15.0) as response:
            return response.read()
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"gateway {path} returned HTTP {exc.code}: {detail}") from exc
    except URLError as exc:
        raise RuntimeError(f"gateway {path} unavailable: {exc.reason}") from exc


def _post(base_url: str, token: str, path: str) -> dict:
    raw = _request(base_url, token, "POST", path)
    return json.loads(raw.decode("utf-8"))


def _capture(
    base_url: str,
    token: str,
    output_dir: Path,
    attempt: int,
    outcome: str,
) -> dict:
    image_path = output_dir / f"attempt_{attempt:02d}_{outcome}.jpg"
    context_path = output_dir / f"attempt_{attempt:02d}_{outcome}_context.json"
    image_path.write_bytes(_request(base_url, token, "GET", "/snapshot"))
    context = json.loads(
        _request(base_url, token, "GET", "/navigation/context").decode("utf-8")
    )
    context_path.write_text(
        json.dumps(context, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return {
        "attempt": attempt,
        "outcome": outcome,
        "image": str(image_path),
        "context": str(context_path),
        "pose": context.get("pose"),
        "clearance": context.get("hard_clearance_m"),
    }


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gateway-url", default="http://127.0.0.1:8090")
    parser.add_argument(
        "--journal", default=str(Path.home() / "hwr" / "state" / "exploration.jsonl")
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--translations",
        type=int,
        default=3,
        help="maximum translation attempts, including failed or timed-out goals",
    )
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--failure-limit", type=int, default=5)
    return parser.parse_args()


def main() -> int:
    args = _args()
    if args.translations <= 0 or args.failure_limit <= 0 or args.timeout <= 0:
        raise SystemExit("translations, failure-limit, and timeout must be positive")

    token = _token()
    journal_path = Path(args.journal)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    event_path = output_dir / "events.jsonl"
    summary_path = output_dir / "summary.json"
    started_at = time.time()
    deadline = time.monotonic() + args.timeout
    translation_attempts = 0
    translations_succeeded = 0
    rotations = 0
    failures = 0
    consecutive_failures = 0
    captures: list[dict] = []
    active_kind: str | None = None
    active_translation_attempt: int | None = None
    reason = "monitor_error"
    stop_results: dict[str, object] = {}

    # Start at EOF so only this sector's events are counted.
    with (journal_path.open("r", encoding="utf-8") as journal,
          event_path.open("a", encoding="utf-8") as event_log):
        journal.seek(0, os.SEEK_END)
        try:
            start_result = _post(args.gateway_url, token, "/explore/start")
            print(json.dumps({"event": "sector_started", "result": start_result}), flush=True)
            while time.monotonic() < deadline:
                line = journal.readline()
                if not line:
                    time.sleep(0.1)
                    continue
                event_log.write(line)
                event_log.flush()
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                name = event.get("event")
                kind = event.get("kind")
                if name == "goal_selected":
                    active_kind = kind
                    if kind == "translation":
                        translation_attempts += 1
                        active_translation_attempt = translation_attempts
                        print(json.dumps({
                            "event": name,
                            "kind": kind,
                            "translation_attempts": translation_attempts,
                            "distance": event.get("preflight", {}).get("distance"),
                            "waypoint": event.get("waypoint"),
                        }), flush=True)
                    else:
                        active_translation_attempt = None
                elif name == "goal_succeeded":
                    consecutive_failures = 0
                    if kind == "rotation":
                        rotations += 1
                        print(json.dumps({"event": name, "kind": kind,
                                          "rotations": rotations}), flush=True)
                    elif kind == "translation":
                        translations_succeeded += 1
                        attempt = active_translation_attempt or translation_attempts
                        capture = _capture(
                            args.gateway_url, token, output_dir, attempt, "success"
                        )
                        captures.append(capture)
                        print(json.dumps({"event": name, "kind": kind,
                                          "translation_attempts": translation_attempts,
                                          "translations_succeeded": translations_succeeded,
                                          "pose": capture["pose"]}), flush=True)
                        if translation_attempts >= args.translations:
                            reason = "translation_attempt_target_reached"
                            break
                    active_kind = None
                    active_translation_attempt = None
                elif name == "frontier_failed":
                    failures += 1
                    consecutive_failures += 1
                    failed_translation = active_kind == "translation"
                    if failed_translation and active_translation_attempt is not None:
                        capture = _capture(
                            args.gateway_url,
                            token,
                            output_dir,
                            active_translation_attempt,
                            "failed",
                        )
                        captures.append(capture)
                    print(json.dumps({"event": name,
                                      "reason": event.get("reason"),
                                      "failures": failures,
                                      "consecutive_failures": consecutive_failures,
                                      "failed_translation": failed_translation,
                                      "translation_attempts": translation_attempts}),
                          flush=True)
                    active_kind = None
                    active_translation_attempt = None
                    if failed_translation and translation_attempts >= args.translations:
                        reason = "translation_attempt_target_reached"
                        break
                    if consecutive_failures >= args.failure_limit:
                        reason = "failure_limit_reached"
                        break
                elif name == "session_finished":
                    reason = "exploration_session_finished"
                    print(json.dumps({"event": name,
                                      "reason": event.get("reason")}), flush=True)
                    break
            else:
                reason = "monitor_timeout"
        except Exception as exc:  # The finally block still enforces the stop.
            reason = f"monitor_error: {type(exc).__name__}: {exc}"
            print(json.dumps({"event": "monitor_error", "reason": reason}), flush=True)
        finally:
            for name, path in (("explore_stop", "/explore/stop"),
                               ("stop", "/stop")):
                try:
                    stop_results[name] = _post(args.gateway_url, token, path)
                except Exception as exc:
                    stop_results[name] = {
                        "error": f"{type(exc).__name__}: {exc}"
                    }

    summary = {
        "started_at": started_at,
        "finished_at": time.time(),
        "reason": reason,
        "translation_attempt_target": args.translations,
        "translation_attempts": translation_attempts,
        "translations_succeeded": translations_succeeded,
        "rotations": rotations,
        "failures": failures,
        "consecutive_failures": consecutive_failures,
        "captures": captures,
        "stop_results": stop_results,
    }
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"event": "sector_finished", **summary}), flush=True)
    return 0 if reason == "translation_attempt_target_reached" else 2


if __name__ == "__main__":
    raise SystemExit(main())
