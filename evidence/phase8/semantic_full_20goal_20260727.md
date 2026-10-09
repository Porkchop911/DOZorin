# Full semantic navigation soak - 2026-07-27

Result: **PASS, 20/20 reached** (acceptance threshold: at least 18/20).

- Invocation: seed 44, 20 goals, 10 candidates per normal/escape band, semantic
  preflight enabled, one inference transport retry.
- Total elapsed time: 1581.3 s (26 min 21.3 s), including candidate rotations,
  settling, local-VLM assessments, and final revalidations.
- Candidate audit: 230 sampled; 226 semantic preflights; 160 rejected; 66
  provisionally allowed; 27 final revalidations.
- Outcome audit: 20 reached, 0 safe refusals, 0 drive failures, 0 reverse samples,
  `max_reverse = 0.0`, runner summary `passed = true`.
- Final state: stop re-latched, no active goal, localization tracking, and
  `trapped_start = false`.

The per-goal time below is the dispatched movement duration after semantic and
metric preflight; candidate-search time is included in the total elapsed time.

| Goal | Outcome | Motion time (s) | Final pose `(x, y, yaw)` |
|---:|---|---:|---|
| 1 | reached | 3.5 | (0.875, -0.406, 2.461) |
| 2 | reached | 1.9 | (1.043, -0.590, -0.801) |
| 3 | reached | 2.2 | (1.224, -0.483, 0.635) |
| 4 | reached | 4.3 | (0.627, -0.574, -2.976) |
| 5 | reached | 3.6 | (1.081, -0.765, -0.426) |
| 6 | reached | 2.7 | (0.721, -0.639, 2.858) |
| 7 | reached | 3.9 | (1.229, -0.857, -0.437) |
| 8 | reached | 2.4 | (0.933, -0.820, 3.132) |
| 9 | reached | 2.2 | (0.936, -0.589, 1.556) |
| 10 | reached | 2.2 | (1.227, -0.660, -0.268) |
| 11 | reached | 3.5 | (0.947, -0.474, 2.585) |
| 12 | reached | 3.8 | (1.429, -0.706, -0.324) |
| 13 | reached | 3.9 | (1.032, -0.467, 2.689) |
| 14 | reached | 2.5 | (1.391, -0.513, -0.043) |
| 15 | reached | 3.1 | (1.005, -0.666, -2.755) |
| 16 | reached | 2.7 | (1.327, -0.759, -0.303) |
| 17 | reached | 4.0 | (0.860, -0.623, 2.871) |
| 18 | reached | 4.5 | (1.483, -0.860, -0.388) |
| 19 | reached | 2.7 | (1.222, -0.654, 2.501) |
| 20 | reached | 2.5 | (1.477, -0.500, 0.510) |

Artifacts:

- `semantic_full_20goal_20260727_acceptance2.jsonl` - complete machine journal.
- `semantic_full_20goal_20260727_acceptance2.stdout` - human-readable runner log.
- `semantic_full_20goal_success_20260727.jpg` - stopped final camera frame.
