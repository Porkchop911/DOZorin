# Phase 10 — Gateway, CLI, LLM agent

**Objective.** The command surface: one authenticated HTTP/WS gateway on the robot, a full PC CLI, and the LLM tool server that turns everything built so far into skills an agent can compose. End state of requirement 4.

**Prerequisites.** Phases 8–9 (things worth commanding). Gateway skeleton can land earlier if convenient; the acceptance demo needs nav + arm + camera.

## Subtasks

- [ ] **10.1 hwr_gateway.** FastAPI node (uvicorn in the ROS process, single asyncio loop bridged to rclpy executor): REST — `POST /drive` (short-lived, watchdog-backed), `/stop` (=estop), `/goto {x,y,yaw|place}`, `/explore/start|stop`, `/arm/pose`, `/arm/reach`, `/gripper`, `/snapshot` (JPEG), `GET /status` (pose, battery, mode, loc-health, current goal), `/map.png`; WS `/telemetry` (2 Hz status + events). Static bearer token (config, not committed); bind LAN; every request logged with outcome.
- [ ] **10.2 CLI.** `cli/hwr.py` (click or argparse + requests): full endpoint coverage, `hwr status` one-screen human summary, `hwr watch` tailing the WS. Works from Windows with only `requests`/`websockets` installed.
- [ ] **10.3 hwr_agent — skills layer.** Skill = validated, bounded, composable action with a text-friendly result: `goto_place(name)`, `goto(x,y)`, `explore(minutes)`, `look(direction|bearing)` → photo, `describe_view()` (snapshot → LM Studio VLM `qwen/qwen3-vl-4b` → text), `patrol(places)`, `arm_pose(name)`, `pick_test()`, `battery()`, `where_am_i()` (pose + nearest place + loc health in words). Skills call the gateway (same audited path as humans). Manifest exported two ways: **OpenAI-style tools JSON** (for LM Studio-driven loops) and an **MCP server** (for Hermes/Claude).
- [ ] **10.4 Guardrails.** In the skill layer, not the prompt: battery gate; geofence = goals must be free cells on the known map; per-skill timeouts; max-actions-per-mission; human interrupt (`hwr stop` preempts everything, agent gets "interrupted" result); dry-run mode (skills narrate instead of act) for prompt development.
- [ ] **10.5 Agent loop.** Minimal mission runner: system prompt (robot facts, skill docs, safety lines) + user mission → LM Studio (instruct model — project lesson: reasoning models narrate instead of act; temp ≤0.4) → tool-call loop → final report with evidence (photos taken). Every mission produces a transcript log.
- [ ] **10.6 E2E demo missions.** ① "Go to the window and describe what you see." ② "Check whether the drums are still there, photo as proof." ③ "Explore for 3 minutes and tell me what's new." ④ "Go to the desk, look left, come back, report." ⑤ free-form user-invented mission. Attended, estop at hand.

## Acceptance & evidence

- **Test:** ≥4/5 missions completed unaided (correct sequence of skills, truthful report); interrupt mid-mission works; all actions present in the gateway audit log; dry-run mode demonstrably safe for prompt iteration.
- **Evidence:** `evidence/phase10/`: mission transcripts + photos, audit log excerpt, guardrail demonstration (geofence rejection of an off-map goal).

## Safety / rollback

- The gateway is the only externally reachable surface — token required, LAN only; skills are the only path the LLM has (no shell, no raw topics); guardrails enforced server-side. Missions attended until a later, separately-agreed unattended bar.

**Effort.** 2–3 days.

**Open questions.** Which LM Studio model for the tool loop (start `qwen3.5-9b`; evaluate); Hermes integration timing (MCP server makes it drop-in later — coordinate with the existing `rosorin-robot` skill rather than duplicating it).
