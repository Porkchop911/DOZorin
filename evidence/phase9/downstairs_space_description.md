# Downstairs space description

Status: **evidence-backed continued exploration checkpoint, updated 2026-07-31.** This is
the best grounded description from the current independent-stack run. It is not
yet a 90% safely-reachable coverage claim. Observations, metric estimates,
vision-model interpretations, and unresolved areas are kept separate below.

## Directly observed layout

The explored downstairs area is a connected living/dining space with a
continuous light wooden floor. Furniture divides it into several linked zones:

1. **Living-room zone.** A grey sofa occupies one side. A low, dark coffee
   table stands in front of it, a floor fan sits near the sofa, and a large
   grey-and-black media cabinet/TV unit forms the opposite boundary. The broad
   center floor is traversable, but the fan base, coffee-table legs, sofa feet,
   cabinet feet, and cables under furniture are persistent positive obstacles.
2. **Plant and shelving edge.** Tall open shelving with books/objects and
   several plants bounds the earlier resume location. More plant pots stand at
   the end of the media cabinet beside a white partition. This creates a real
   passage into the center floor, but it is narrower than the visually open
   room around it.
3. **Dining zone.** A table, ordinary chairs, and tall stools create a dense
   field of thin black legs. During the 2026-07-31 route the robot passed beside
   one chair-leg group. The leg moved from the image edge into the forward
   corridor as the robot approached, reducing forward clearance to 0.30 m and
   side clearance to 0.10-0.15 m. The explorer then used bounded rotations to
   select a clearance-increasing direction and moved away without reported
   contact. The continued route later crossed the open aisle between this leg
   field and the plant/cabinet boundary, then returned to a living-room-facing
   viewpoint by another mapped approach.
4. **Kitchen/bar-side aisle.** Views from the earlier sectors show white fitted
   cabinetry/appliance fronts on one side and a taller counter/table with
   stools on the other. The central wooden-floor aisle connects to the
   living/dining floor. Bright exterior light or glazing produces strong glare
   near this side. The continued run physically advanced along the connected
   wooden-floor aisle, but did not claim the bright exterior-facing boundary as
   traversable.
5. **Door and darker passage.** A white door frame and an adjacent open,
   darker passage are visible beyond the cabinet/table transition. The opening
   is visually real, but the robot has not entered or mapped the space beyond
   it. A dark object visible low in that opening has not been identified.

A seated person was visible on the sofa during much of the run. The person was
outside every accepted short swept corridor; no claim is made about identity.
No staircase or unsupported edge was entered or declared traversable.

## Metric evidence

- Map resolution is 0.05 m per cell. The final saved raster is 359 by 311
  cells.
- The continued checkpoint contains 21,016 ray-observed free cells, 1,410
  occupied cells, and 89,223 unknown cells. Free-cell area is approximately
  **52.54 m^2** and occupied-cell area approximately **3.53 m^2**. These are raster cell areas,
  not floor-plan or safely reachable areas: sensor rays see behind and between
  furniture, while the inflated robot footprint removes additional space.
- From the resume-canary checkpoint to the terminal map, observed free space
  increased from 17,853 to 20,883 cells, a gain of 3,030 cells or about
  **7.58 m^2** of ray-observed free-cell area.
- Continuing from the earlier `(2.58, 3.66)` terminal pose added 133 free cells
  (about **0.33 m^2**) and 79 occupied cells while converting another 212
  unknown cells. The smaller area gain is consistent with driving through
  space that long sensor rays had already observed, while directly validating
  its traversability and nearby furniture boundaries.
- The full 2026-07-31 route resumed near map pose `(1.05, 2.49)` and the latest
  saved state is near `(3.52, 4.25)`, a net map-frame displacement of about
  **3.05 m**. The continued portion alone displaced about **1.12 m** from its
  earlier checkpoint. Scan
  matching can correct the map pose, so this is not a tape-measure path length.
- Sectors 12 through 20 added 14 monitored translation attempts totalling
  about **1.265 m** requested; 11 succeeded. Together with the earlier run, the
  journal now contains 34 monitored attempts, about **3.30 m** requested, and
  26 successes. The continued sectors recorded 66 bounded rotation stages
  (including the one pre-crash canary turn) and 22 frontier-failure events.
- Post-checkpoint sectors 21 and 22 added five more successful translation
  attempts, about **0.432 m** requested, and 15 bounded rotation stages with no
  navigation failures. The combined journal through sector 22 therefore has
  39 monitored translation attempts, about **3.73 m** requested, and 31
  successes. These two sectors were not included in the last saved raster, so
  their coverage is supported by pose-linked image/context evidence rather
  than claimed as saved-map area.
- The last completed live pose in sector 22 was near `(3.61, 4.67)`, about
  **3.37 m** map-frame displacement from the session's `(1.05, 2.49)` resume
  pose. Sector 23 was started only to validate floor-support recovery and was
  interrupted by the human's shutdown; it contributes no claimed result here.
- The latest stopped context reported censored-clear forward space beyond
  1.5 m, about 1.0 m front-left, censored-clear left, 0.30 m front-right, and
  0.35 m right at the final failed-leg capture. That 0.069 m leg advanced about
  0.042 m by odometry, roughly 3 mm below the independent confirmation minimum,
  so it was correctly not counted as success.
- No collision or physical contact was reported or visible in this continued session.
  The final stop is latched and exploration is disabled.

The metric map is useful for connectivity and checkpointing, but its long
ray-shaped lobes and repeated heading corrections mean its outer boundary is
not yet an accepted architectural floor plan.

## Navigation-relevant constraints

- Thin chair, table, stool, cabinet, and fan legs are the dominant local
  hazards. Image-level openness is not sufficient; depth/lidar inflation and a
  fresh floor-support profile remain mandatory.
- The sofa/furniture side contains visible cables and power equipment and is
  treated as a persistent hazard zone.
- Plant pots and cabinet corners form pinch points between otherwise open
  floor regions.
- Windows, glazed doors, mirrors, and other reflective planes are now explicit
  optical boundaries. Mapping drops hit endpoints at or beyond 4.0 m instead
  of carving long free rays through glass; the VLM separately reports visible
  glass and whether it closes the proposed corridor. A corridor glass result
  deterministically vetoes translation and cools both the frontier and its
  approach bearing for 3,600 seconds. Those exclusions persist across bounded
  sector starts and are reconstructed from the journal after a process restart.
- Invalid or stale depth-derived floor support no longer terminates the whole
  exploration session. It still vetoes every translation, but safe in-place
  rotation remains available so the camera can turn away from sunlit glass and
  re-establish a valid floor plane. This recovery contract passed the full 286
  test suite and was deployed before the interrupted sector-23 canary.
- This guard was exercised live five times in sectors 17 through 20. One
  representative result identified a “bright light source through transparent
  door”; metric clearance was otherwise clear, but translation was held and
  exploration redirected to another frontier.
- A 0.20 m clearance margin proved too exact beside the chair. The explorer
  now requires a 0.25 m semantic/metric margin beyond each requested leg.
- Translation attempts are permanently capped at 0.10 m. Navigation now has a
  six-second short-leg deadline plus an odometry leash that aborts if physical
  travel exceeds the requested leg by more than 0.05 m.
- Reverse motion remains structurally unavailable. Close furniture must be
  handled by a safe rotation and forward/holonomic escape direction.
- When scan matching placed the stopped pose inside the plant-pot inflation
  plateau, costmap-monotonic escape checks rejected two unsafe/no-path strafe
  proposals at zero motion. A camera/VLM-inspected right sector and a bounded
  in-place heading adjustment returned the pose to free cost before planned
  exploration resumed.

## Local vision-model interpretations

The local vision-language model repeatedly classified the immediate 0.10 m
wood-floor envelopes as clear and classified the seated person as outside the
accepted corridor. It also described the visible opening as a doorway/passage
and distinguished ordinary open doorways from bright transparent boundaries.
Those interpretations were combined with metric clearance and floor-support
checks; they never overrode a geometric rejection. A visible doorway, cabinet,
person, or piece of furniture is not a verified semantic landmark or proof of
physical arrival.

## Unresolved areas and remaining work

- The darker space beyond the open passage remains unentered and undescribed.
- The kitchen/bar side now has additional pose-linked aisle views, but its
  exterior-facing glass boundary and spaces beyond it are intentionally not
  mapped as reachable interior floor.
- The star-shaped outer map boundary requires loop/relocalization validation
  before dimensions or a safely reachable coverage percentage are accepted.
- Pinning each local approach waypoint across rotation stages removed the
  earlier alternating-heading oscillation, but total yaw staging remains high
  when many distinct furniture/glass frontiers must be rejected.
- Safely reachable coverage, inflated-costmap area, frontier replay, and the
  90% target have not yet been calculated.
- Stable room/door/furniture labels may later become topological landmarks, but
  no object-arrival anchor is valid until a direct approach proves the robot is
  within 0.50 m of the actual object.
- The user-deferred dynamic-obstacle physical trial remains skipped.

## Evidence index

- Earlier map checkpoint: `downstairs_session_final_20260731.{pgm,yaml,state.yaml}`
- Continued checkpoint: `downstairs_session2_20260731.{pgm,yaml,state.yaml}`
- Continued gateway rendering: `downstairs_session2_map_20260731.png`
- Exact 2026-07-31 path overlay:
  `downstairs_session_trajectory_annotated_20260731.png`
- Resume evidence: `downstairs_resume_canary_20260731.*`
- Bounded sector evidence: `downstairs_sector3_20260731/` through
  `downstairs_sector22_20260731/` (with descriptive suffixes on canary/escape
  folders)
- Latest completed views and contexts:
  `downstairs_sector22_20260731/attempt_01_success.*` and
  `downstairs_sector22_20260731/attempt_02_success.*`
- Window guard evidence: `downstairs_sector17_20260731/events.jsonl` through
  `downstairs_sector20_20260731/events.jsonl`
- Earlier panorama and sectors: `downstairs_panorama_*_20260730.jpg`,
  `downstairs_sector1_20260730/`, and `downstairs_sector2_20260730/`
