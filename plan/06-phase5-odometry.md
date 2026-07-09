# Phase 5 — Hand-rolled odometry (scan matching + fusion)

**Objective.** Trustworthy `odom→base_footprint` without encoders: our 2D correlative scan matcher as the displacement source, fused with the velocity model and IMU. This is the load-bearing phase — SLAM, MCL, and navigation all stand on it. The correlative core built here is reused in Phases 6 and 7.

**Prerequisites.** Phases 3–4 (bags with scan+imu+odom_model). Develop offline on bags first (convention), then live.

## Subtasks

- [x] **5.1 Correlative scan matcher core.** `hwr_odom/matcher.py` (ROS-free, lives importable by hwr_slam/loc too): rasterize reference scan into a smoothed likelihood grid; search Δx,Δy,Δθ coarse-to-fine (e.g. ±0.15 m/±10° coarse 3 cm/1°, refine 0.5 cm/0.25°); numpy-vectorized scoring; returns pose + score + covariance estimate from the score surface. Unit tests: synthetic scans with known offsets, degenerate cases (corridor → high cross-track variance flagged).
- [ ] **5.2 Scan-odometry node.** Match scan(k) against a keyframe (re-key on >0.3 m/20° or score decay), ~5 Hz, publish `odom_scan` (twist + pose deltas, covariance from matcher). Budget: <30% of one Jetson core (profile; shrink search window using the model-odom prior — seeded search is both faster and more robust).
- [ ] **5.3 Fusion filter.** `hwr_odom/fusion.py`: our small 2D EKF (state x,y,θ,vx,vy,ω): predict with model odom (cmd-vel including vy) + IMU ω; update with `odom_scan` deltas gated by score. Publishes fused `odom` + owns the `odom→base_footprint` TF (hwr_base's model-only TF turns off via param). Unit tests with scripted scenarios (slip: model says moving, scan says stuck → fused follows scan).
- [ ] **5.4 Offline validation.** Run matcher+fusion over the Phase 3/4 bags; plot trajectories: model-only vs fused vs (for context) whatever the vendor EKF recorded. Tune windows/gates. Target visibly: rotation without divergence, strafe visible in odom (vendor's isn't).
- [ ] **5.5 Live acceptance.** Tape a 2×2 m square on the floor; teleop 3 laps ending on the start mark; record fused odom. Also 30 s continuous in-place rotation, then drive 1 m — no divergence.

## Acceptance & evidence

- **Test:** loop closure error ≤5 cm / 5° per lap (3/3 laps); rotation test passes; matcher CPU within budget; slip scenario unit test green.
- **Evidence:** `evidence/phase5/`: trajectory plots (model vs fused, bag + live), loop-closure numbers table, CPU profile note.

## Safety / rollback

- All motion is attended teleop. Fusion TF behind a param — flip back to model odom instantly if fused TF ever misbehaves mid-development.

**Effort.** 2–3 days (the matcher is the single most valuable piece of code in the repo — spend the time here, not in Phase 6 heroics).

**Open questions.** Is 10 Hz lidar + 5 Hz matching enough during fast rotation (scan distortion)? If skew is visible, de-skew scans with IMU ω before matching (add as 5.6 if needed).

## Status (2026-07-09) — 5.1 done ahead of Phases 3/4 motion items

Built during the movement pause (pure PC work): `hwr_core/matching/correlative.py` — likelihood-field grid (Gaussian smear, bilinear sampling), coarse-to-fine (±15 cm/±10° @ 3 cm/1° → ±3 cm/±1° @ 0.5 cm/0.25°), prior-seeded window, covariance from the **coarse** score surface (fine samples cluster at the peak and would hide flat directions), corridor degeneracy flag (eig-ratio > 16). Lives in hwr_core (not hwr_odom) so SLAM/MCL import it without ROS — deliberate deviation from the 5.1 path spec.

11 unit tests vs deterministic sim2d scans: known offsets ±1.5 cm/0.75° (with 1 cm range noise: 2 cm/1°), base-frame delta convention, prior extends range past the window, corridor flagged degenerate with along-axis σ > 3× cross-axis, mismatched scene scores < 0.6 vs > 0.85 matched. 90/90 suite green.

Timing (PC, 380-pt scan): set_reference 22 ms; match 74 ms full window, **33 ms prior-seeded** (±6 cm/±4°) — 5.2's seeded search is the production path; profile on the Jetson before accepting the <30 %-of-one-core budget.

**Note for 5.4/5.5:** real-bag validation and live acceptance still gated on Phase 3/4 movement items (bags don't exist yet).
