# Phase 7 — Hand-rolled localization (MCL)

**Objective.** Know where we are on a saved map: our particle filter with a likelihood-field sensor model, plus brute-force global relocalization for cold start and kidnap recovery, with an honest health signal.

**Prerequisites.** Phase 6 map (`hwr_map_01`); Phase 5 fusion (motion model + matcher core).

## Subtasks

- [ ] **7.1 Likelihood field.** Precompute distance-transform of the saved map → Gaussian likelihood lookup (scipy.ndimage). Shared with the matcher (same rasterization as 6.2 — one code path).
- [ ] **7.2 Particle filter.** `hwr_loc/mcl.py` (ROS-free, numpy): N≈500 particles; motion model = fused-odom delta + tunable noise (per-axis, boosted while rotating); weight = likelihood-field score of a decimated scan (~60 beams); low-variance resampling; adaptive N optional later. Unit tests on synthetic map+scans (convergence from spread init, tracking under noise).
- [ ] **7.3 Global relocalization.** Coarse correlative search of the current scan over the map's free space (the proven `global_loc.py` approach from the old stack, reimplemented on our matcher core): score grid of candidate poses → seed MCL cloud at the top hypotheses. Runs on demand (service) and automatically when health collapses.
- [ ] **7.4 Localizer node.** Publishes `map→odom` TF (replacing the mapper's when in localization mode), `pose` with covariance, and **health**: effective sample size + mean scan score on a topic; mode services (localize-on-map vs mapping handled by a small mode manager so SLAM and MCL never both own `map→odom`).
- [ ] **7.5 Offline tests.** Bags replayed against `hwr_map_01`: convergence time from unknown pose, tracking error along a teleop tour (visual RViz + score plot).
- [ ] **7.6 Live acceptance.** ① cold start anywhere → global loc → converged (verify with the scan-overlay-on-map plot, the old stack's proven evidence style); ② kidnap: carry the robot ≥3 m mid-run, 3 trials; ③ full teleop tour without losing lock.

## Acceptance & evidence

- **Test:** kidnap recovery unaided in <30 s, 3/3 trials; tour tracked end-to-end (health above threshold throughout); cold-start convergence <20 s.
- **Evidence:** `evidence/phase7/`: scan-vs-map overlay images before/after each recovery, health-metric time series plot.

## Safety / rollback

- Attended teleop; localization has no motion authority. Health topic is what Phase 8 will gate motion on — design it honestly here.

**Effort.** 1.5–2 days.

**Open questions.** Particle count vs CPU with everything else running (profile at 7.5); whether MCL even needs to run when the matcher's scan-to-map lock is strong (possible later optimization: matcher-primary, MCL as hypothesis keeper).
