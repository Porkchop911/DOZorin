# Phase 7 — Hand-rolled localization (MCL)

**Objective.** Know where we are on a saved map: our particle filter with a likelihood-field sensor model, plus brute-force global relocalization for cold start and kidnap recovery, with an honest health signal.

**Prerequisites.** Phase 6 map (`hwr_map_01`); Phase 5 fusion (motion model + matcher core).

## Subtasks

- [x] **7.1 Likelihood field.** Precompute distance-transform of the saved map → Gaussian likelihood lookup. Implemented in `hwr_core.localization` and covered by synthetic-map tests.
- [x] **7.2 Particle filter.** ROS-free numpy MCL with 500 particles, fused-odom prediction, decimated scan likelihoods, and low-variance resampling. Synthetic convergence/tracking tests pass.
- [x] **7.3 Global relocalization.** Coarse scan correlation seeds the strongest hypotheses; available on demand and after sustained health collapse. Live cold-start score 0.760 on 2026-07-22.
- [x] **7.4 Localizer node.** Sole `map→odom` owner in localization mode; publishes pose and ESS/scan-score health. `deploy/mode.sh` now performs exclusive process-group mode swaps.
- [ ] **7.5 Offline tests.** Bags replayed against `hwr_map_01`: convergence time from unknown pose, tracking error along a teleop tour (visual RViz + score plot).
- [ ] **7.6 Live acceptance.** Cold start and short-motion tracking passed on 2026-07-22 (`evidence/phase7/live_localization_20260722.md`). Still open: scan-overlay plot, three ≥3 m kidnap trials, and full teleop tour.

## Acceptance & evidence

- **Test:** kidnap recovery unaided in <30 s, 3/3 trials; tour tracked end-to-end (health above threshold throughout); cold-start convergence <20 s.
- **Evidence:** `evidence/phase7/`: scan-vs-map overlay images before/after each recovery, health-metric time series plot.

## Safety / rollback

- Attended teleop; localization has no motion authority. Health topic is what Phase 8 will gate motion on — design it honestly here.

**Effort.** 1.5–2 days.

**Open questions.** Particle count vs CPU with everything else running (profile at 7.5); whether MCL even needs to run when the matcher's scan-to-map lock is strong (possible later optimization: matcher-primary, MCL as hypothesis keeper).
