# Conventions — how work on this repo is done

## Safety (non-negotiable)

1. **Bench-first.** The first time any new code path can move an actuator, the robot is on a stand with wheels off the ground (motors) or in open space with the arm clear (servos). Floor tests only after bench behavior is understood.
2. **Battery discipline.** Check `/ros_robot_controller/battery` (or our equivalent) at session start and end. <11000 mV: wrap up. <10500 mV: stop immediately. Never charge powered on.
3. **No reverse, ever.** The robot is rear-blind (lidar ~140° aft blocked). No controller, recovery, or teleop default may command negative vx unless a rear-sensing story exists first.
4. **Watchdog before power.** Any node that can command motion ships its command watchdog (auto-stop on stale input) in the same commit — never later.
5. **Attended actuation.** Until Phase 8's soak test, every run that can move the robot is attended with a hand on the estop path.
6. **Takeover hygiene.** Hardware access only between `takeover.sh` and `handback.sh`. Never kill `button_scan` (it legitimately shares ttyACM0). Always hand back before ending a session.

## Evidence-gated progress

- Every phase acceptance test defines its artifact (photo, plot, log, bag) up front; artifacts land in `evidence/phaseN/` with a one-line README entry.
- A checkbox in a subplan flips **only** when the artifact exists. "It should work" ≠ done. Rates are reported with denominators (3/3, 18/20), never percentages of task lists.
- If the user finds a problem we didn't report first, that's a monitoring failure — log it in the phase's subplan under "incidents".

## Code

- **Jazzy-ready**: pure `rclpy` + std interfaces; no Humble-only APIs; launch files in Python with no distro-conditionals; parameters in YAML; no reliance on message fields deprecated after Humble. Target: the workspace builds on Jazzy in a container unchanged (tracked in ADR-001).
- **hwr_core stays ROS-free.** Protocol, kinematics, safety logic import nothing from ROS; they get unit tests that run on the Windows PC (`pytest hwr_core`).
- **Clean-room rule**: vendor code is read as spec, cited in comments as `ref: 90-clean-room-reference.md §X`, never copied. The CRC table is data, not code — hand-transcribing it is fine.
- Algorithmic code (scan matcher, MCL, A*, controller) gets PC-side unit tests + bag/sim validation **before** robot time.
- Style: numpy-vectorized where it matters (Jetson CPU is the scarce resource — the vendor stack already proved pointclouds can starve it), type hints, small modules, no comment noise.

## Workflow

- PC is the source of truth; `deploy/sync` pushes to the robot at `~/hwr`. No editing on the robot except throwaway experiments — anything kept gets synced back the same session.
- Remote execution uses the base64→zsh SSH pattern (see `90-clean-room-reference.md` §Ops).
- Each phase = one or more commits with the phase number in the message; ROADMAP updates (checkbox flips + evidence links) commit together with the work.
- `plan/*.md` files are living documents: findings, deviations, and incidents are appended to the relevant subplan, not scattered.
