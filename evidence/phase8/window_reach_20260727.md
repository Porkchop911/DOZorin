# Window reach acceptance — 2026-07-27

- Requirement: visibility is insufficient; robot center must be no more than
  0.50 m from the physical object.
- A right-edge attempt correctly aborted after the live costmap exposed the
  rack/chair obstruction.
- The operator identified a free left avenue. A 0.65 m staged leg and the final
  0.32 m leg both succeeded with localization healthy and stop re-latched after
  each action.
- Terminal localized pose: approximately `(1.986, -0.976)`.
- Close-range window ROI depth: p25 `0.388 m`, median `0.408 m`, p75 `0.422 m`
  with 99.3% valid pixels.
- The close-range window point projected through live
  `map -> depth_camera_link` is `(2.289, -1.220)`.
- Configured approach `(2.025, -1.004)` is `0.341 m` from that measured point.
- Result: **window reached** under the strict `<= 0.50 m` rule.
- Deployed named-place acceptance returned `state: succeeded`,
  `object_reached: true`, and `object_distance_m: 0.385775` before stop was
  re-latched.

Frames:

- `window_left_avenue_preflight_20260727.jpg`
- `window_left_avenue_stage_20260727.jpg`
- `window_reach_0521m_20260727.jpg`
- `window_named_reached_20260727.jpg`
