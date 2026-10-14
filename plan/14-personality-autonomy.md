# Phase 12 — Soudruhu: body, senses, personality, free roam

**Objective.** The robot becomes one continuous character driven by Hermes (`hermes -p robot`, Strata only): it speaks through its own speaker, sees and tracks objects on the Jetson, hears its wake word, and roams the mapped area on its own, remembering what it meets.

**Decided by the user (2026-10-13).**
- Persona: **Soudruhu**, a comrade. Answers the wake word with "Allzeit bereit!". Jovial, strong sense of justice, vehemently antifascist in an Eastern-bloc way, Boxer-like (Animal Farm) unwavering dedication to his people, relaxed about surveillance.
- Idle behaviour: **free roam** within the safety envelope.
- Voice: **neural TTS on the Jetson** (sherpa-onnx `vits-ljs`, already installed), out of USB audio card 0.
- Listening: **wake word always on** while the stack runs (overrides the earlier "never start the mic unprompted" rule for this mode).
- Hermes has a shell on the Jetson (approvals off, hardline blocklist kept), web access, raw ROS via ros-mcp, and resets its own estop.

**Prerequisites.** HWR navigation mode on a saved map; PC relay `tools/lm_studio_relay.py --target-port 8080` for explorer preflights; Strata on :8080.

## Subtasks

- [ ] **12.1 Voice out.** `hwr_voice` on the robot: sherpa-onnx VITS synth → `aplay -D plughw:0,0`; one-at-a-time queue, interrupt on new utterance. Gateway `POST /say {text}` (≤400 chars) and agent tool `say(text)`. No buzzer anywhere (standing rule).
- [ ] **12.2 Eyes.** `hwr_vision` node: Ultralytics YOLO11n exported to TensorRT FP16 on the Orin (ultralytics 8.4.9, TRT 10.7 already present), built-in ByteTrack, at ≥10 Hz on the colour camera. Publishes tracked detections (class, id, bbox, depth-derived range/bearing). Gateway `GET /vision/objects`; agent tool `seen_objects()` returning a text list ("person id 3, 1.2 m, 20° left"). Measure FPS + GPU/CPU load with nav running; fail the subtask if nav loop timing degrades.
- [ ] **12.3 Ears.** Port the July wake-word pipeline (`~/assistant.py`: arecord → sherpa streaming spotter "Soudruhu" → "Allzeit bereit!" → Whisper base.en) onto the HWR stack as `hwr_ears`. Transcripts go to Hermes' API server on the PC (`hermes -p robot gateway`, `api_server` platform) so the spoken turn joins the same memory/session; reply is spoken via 12.1. Amplitude gate + junk filter kept.
- [ ] **12.4 Persona.** Rewrite the robot profile `SOUL.md` around Soudruhu (character above + body/tool rules). Short spoken replies (1–2 sentences), says what it is doing before moving. Seed Hermes memory with the people/places it knows.
- [ ] **12.5 Free roam ("life loop").** PC-side loop that wakes Hermes every N minutes with a short "you are idle, live your life" prompt in a persistent session: pick a reachable free point (new skill `wander()` using `reachable_goal_candidates` on the live map), go, `seen_objects` + `describe_view`, remember notable changes, maybe comment aloud. Any human speech preempts the loop. Rate limits: max speech per hour, max roam legs per hour.
- [ ] **12.6 Boot.** Voice, vision, ears, rosbridge and the relay start with the stack (mode.sh process group / PC scheduled task), so a reboot restores the whole character.

## Acceptance & evidence

- **Test:** 30-minute attended free-roam session: robot roams ≥10 legs with zero contacts, speaks ≥3 relevant in-character remarks, answers ≥4/5 wake-word questions correctly by voice, names ≥3 tracked objects correctly (checked against photos), and its memory afterwards contains at least one correct new fact about the room.
- **Evidence:** `evidence/phase12/`: session transcript, roam trajectory over map, photo/caption pairs, vision FPS/load table, audio clips of 3 replies.

## Safety / rollback

- Motion stays inside the gateway envelope (no reverse, ≤0.2 m/s, preflight + metric clearance). Raw ROS publishing and shell access are user-granted and bypass that envelope; every agent shell change is logged in `~/hwr/state/agent_changes.log`.
- Human `hwr stop` is no longer final (agent may reset); the hard stop is `mode.sh stop` / factory handback.
- Rollback: `hermes -p robot` profile is separate from the default; factory stack restorable with `deploy/handback.sh`.

**Order.** 12.1 → 12.2 → 12.4 → 12.5 → 12.3 → 12.6 (voice and eyes first so the roam has something to say and see; ears last because it needs the Hermes API server).

## Session log — 2026-10-14

- [x] 12.1 voice: `/say` + `say`, single English VITS voice; motto pronounced via two lexicon lines added on the robot (`allzeit`, `bereit`; original kept as `lexicon.txt.orig`, old espeak wake reply as `~/allzeit_espeak.wav`). Must play via PulseAudio (`-D pulse`).
- [x] 12.2 eyes: YOLO11s TensorRT FP16 (`~/third_party/yolo11s.engine`), ByteTrack, 38 ms/frame. Reliable on books; misses the office chair from floor height; called draped fabric "bed".
- [x] 12.4 persona: Soudruhu SOUL.md in the `robot` Hermes profile; motto only via the wake reply.
- [~] 12.3 ears: `tools/ears.py` → Hermes API (`hermes -p robot gateway`, :8643 via relay PC:12341). "turn right" by voice worked end to end. Weak: Jetson fan noise drowns soft speech (noise floor peak ~800 at 100 % gain; speech peaked ~3100). Next: noise suppression + noise-floor-relative threshold.
- [ ] 12.5 free roam — **acceptance FAILED** (30 min, 10 ticks): 2/7 planned legs succeeded, 0 contacts, 2 remarks aloud, correct new memory (drum kit). Root cause: localizer rejects scan matches as "inconsistent with odometry" (269×) during any planned translation, even in open floor → loc 0.95→0.25 → nav aborts. Next: measure odometry (bounded in-place turn + 0.3 m leg: wheel vs gyro vs scan-matched), fix, then add a `relocalize` tool (`/global_relocalize` took 0.37→0.96). Explore is unavailable in navigation mode (503).
- [ ] 12.6 boot: still manual — takeover, mode.sh navigation, rosbridge, ears, PC relays (:12340→8080, :12341→8643), `hermes -p robot gateway`.
