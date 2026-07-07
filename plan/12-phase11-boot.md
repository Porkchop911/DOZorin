# Phase 11 — Boot story, hardening, audit

**Objective.** Make the rework a first-class resident of the robot: optional boot service, a failure runbook, and an honest final audit of every phase gate. The factory stack remains installed and switchable forever.

**Prerequisites.** Phases 0–10 accepted.

## Subtasks

- [ ] **11.1 hwr.service.** Systemd unit: waits for devices, runs takeover + our bringup launch (driver, sensors, odom, localizer, nav, gateway) under the `ubuntu` user, journal logging. **Disabled by default** — installed but the user explicitly chooses the boot default (`hwr boot enable|disable` flips between `hwr.service` and `start_app_node.service`; never both).
- [ ] **11.2 Crash resilience.** Restart policies per node (driver restarts must re-latch estop safely, i.e. boot into stopped state); gateway supervises node liveness into `/status`; watchdogs re-verified under kill -9 of each critical node while driving (bench).
- [ ] **11.3 Runbook.** `docs/RUNBOOK.md`: symptoms → causes → commands for the known failure modes (serial wedged → fuser -k + restart; localization lost → global reloc service; costmap imprisonment → recovery/clear; LM Studio unreachable → model pinning `lms load … -y`; factory-stack restore).
- [ ] **11.4 Docs pass.** README quickstart (PC + robot from zero), PROTOCOL.md completeness vs implementation, ADRs updated with outcomes (incl. ADR-005 decision, 4.4 stretch result).
- [ ] **11.5 Final audit.** Walk every subplan: each checked box has its evidence artifact; unchecked boxes get an explicit disposition (done-later / dropped-because). Update the masterplan phase table to final state. Battery + hardware condition check.

## Acceptance & evidence

- **Test:** cold boot → (user-chosen default comes up by itself) → one command switches stacks either direction → run a mission → handback/switch back → factory intact. Kill-test matrix all-green.
- **Evidence:** `evidence/phase11/`: boot journal excerpt, switch-both-ways transcript, kill-test table, final audit note.

## Safety / rollback

- Boot default flip is reversible with one command and documented in the runbook; `hwr boot disable` restores the factory default from any state.

**Effort.** 1 day.

**Open questions.** Whether unattended operation (auto-start missions, patrols on schedule) is wanted at all — explicitly out of scope here; propose separately after the soak history says the stack has earned it.
