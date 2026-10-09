# Live localization acceptance — 2026-07-22

Map: `hwr_map_20260722_live.yaml` (5 cm/cell, 200×200 cells; observed
footprint approximately 8.0×3.7 m).

- Mapping was stopped before localization so only one node owned `map→odom`.
- Global relocalization seeded eight hypotheses; best coarse score: **0.760**.
- Settled health: `tracking`, mean scan score **0.734**, ESS **461/500**.
- After an attended 45° in-place turn: `tracking`, score **0.729**, ESS
  **252/500**.
- After returning to the original heading: `tracking`, score **0.757**, ESS
  **392/500**.
- After the first autonomous navigation move: `tracking`, score **0.771**,
  ESS **236/500**.
- Canonical navigation-mode restart removed the old process group, restored one
  driver/base/localizer/nav set, and relocalized with best coarse score
  **0.860**; settled score **0.770**, ESS **320/500**.

Image evidence: `localization_turn_20260722.jpg`.

This accepts cold-start correlation and short-motion tracking for the current
partial map. The Phase 7 kidnap-recovery gate (three ≥3 m trials) and a full
teleop tour remain open.
