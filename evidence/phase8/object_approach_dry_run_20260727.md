# Object-centric approach dry-run — 2026-07-27

## Implemented

- Point and finite-segment target geometry.
- Deterministic sampling of approach poses 0.30–0.48 m from the physical target.
- Live-costmap geofence, A* reachability, path length, clearance ranking, and
  target-facing yaw.
- Optional local-VLM surface grounding fused with metric depth and map TF.
- Independent marked-image semantic audit, with grid/crop relocation for a
  rejected coordinate.
- Point-to-segment terminal arrival verification in the named-place gateway.

## Stationary live validation

The robot stayed stopped; the runner reported `motion_dispatched: false`.

Using the verified window anchor from live pose approximately
`(1.000, -0.507, -2.520)`, the planner returned five reachable candidates. The
best was `(1.951, -1.025)` facing `-30°`, 0.390 m from the measured window
surface, with a 1.157 m A* path. Every returned candidate was within the strict
0.50 m arrival bound.

The VLM correctly named the turquoise drying-rack foot but initially placed its
coordinate on the connected pole. The marked-image audit rejected that point.
A later grid/crop pass selected cell C5 and moved the proposal downward, but
the final marker audit still could not prove that the pixel lay on the requested
rubber material, so grounding failed closed and no candidate was dispatched.
This demonstrates why recognition confidence alone is not treated as physical
grounding evidence.

## Verification

- Full local suite: 224 passed.
- The robot network endpoint stopped responding during a later stationary
  large-object check. No live motion trial was started.
- The final segment-arrival changes remain local until the next successful sync.
