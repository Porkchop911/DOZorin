# Phase 0 — Repo scaffold, dev loop, hardware switchover

**Objective.** A repeatable edit-on-PC → run-on-robot loop, and proven scripts that take exclusive ownership of the hardware from the factory stack and give it back intact. Nothing here talks RRC yet.

**Prerequisites.** Robot on and charged (>11.5 V to start a session comfortably); SSH key auth working; this plan approved.

## Subtasks

- [ ] **0.1 Scaffold.** Create the code tree from `00-masterplan.md` §layout (empty `__init__.py`/`package.xml` stubs only where needed to make structure real; no logic). Add `evidence/` with per-phase folders. Commit: `phase0: scaffold`.
- [ ] **0.2 Sync script.** `deploy/sync.ps1` (PowerShell, uses scp -r or rsync via Git Bash) + `deploy/sync.sh`: push repo → robot `~/hwr`, excluding `.git`, `evidence/`, `build/`. Idempotent, prints a file-count/size summary. A `deploy/remote.sh <cmd>` helper runs a command on the robot via the base64→zsh pattern (ref `90` §Ops).
- [ ] **0.3 takeover.sh** (runs on robot): ① read battery from the factory topic and refuse if <11000 mV; ② `sudo systemctl stop start_app_node.service`; ③ wait until no process holds `/dev/ttyACM0` except `button_scan` (`fuser -v`), timeout 15 s; ④ write a state file `~/hwr/.takeover` with timestamp; ⑤ print OK. Must be idempotent (re-running while taken over is a no-op).
- [ ] **0.4 handback.sh**: ① stop any hwr processes (pkill pattern on our node names, guarded); ② `sudo systemctl start start_app_node.service`; ③ poll until `/scan` and `/ros_robot_controller/battery` publish again (timeout 60 s) and `curl -s localhost:8080` answers; ④ remove `.takeover`; ⑤ print OK with elapsed time. Idempotent.
- [ ] **0.5 Session log.** Both scripts append one line (timestamp, battery, result) to `~/hwr/switchover.log`; sync pulls that file back for evidence.
- [ ] **0.6 Acceptance run.** Three consecutive takeover→handback cycles in one session; after each handback verify factory topics + web video; capture the log + a `ros2 topic hz /scan` sample.

## Acceptance & evidence

- **Test:** 3/3 takeover→handback cycles clean; factory stack fully functional after each (scan flowing at ~10 Hz, battery topic alive, `:8080` serving).
- **Evidence:** `evidence/phase0/switchover.log` + terminal transcript of the topic checks.

## Safety / rollback

- Worst case is "factory stack won't restart": recovery = `sudo systemctl restart start_app_node.service`; if wedged, `sudo fuser -k /dev/ttyACM0` then restart (known-good recovery from project history). Document in the runbook if ever needed.
- Nothing here commands motion. Battery gate present from the first script.

**Effort.** ~half a day including the acceptance run.

**Open questions.** none.
