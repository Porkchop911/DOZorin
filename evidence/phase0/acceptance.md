# Phase 0 acceptance — 2026-07-07

**Test (per `plan/01-phase0-devloop.md`):** 3 consecutive takeover→handback cycles in one
session; factory stack fully functional after each.

**Result: PASS — 3/3 cycles clean.**

## Switchover log (`switchover.log`, pulled from robot)

```
2026-07-07 10:22:56 takeover FAIL reason=no-battery-reading   <- pre-fix run (set -u bug, see note)
2026-07-07 10:24:54 takeover OK battery=11607mV               <- cycle 1
2026-07-07 10:25:11 handback OK battery=11592mV elapsed=10s
2026-07-07 10:25:25 takeover OK battery=11587mV               <- cycle 2
2026-07-07 10:25:37 handback OK battery=11607mV elapsed=9s
2026-07-07 10:25:42 takeover OK battery=11584mV               <- cycle 3
2026-07-07 10:25:54 handback OK battery=11593mV elapsed=9s
```

Idempotency verified between cycle 1 takeover and handback: second `takeover.sh`
invocation printed `already taken over since 2026-07-07 10:24:54 - no-op` and exited 0.

## Factory health after final handback

```
$ ros2 topic hz /scan
average rate: 9.957   min: 0.081s max: 0.120s std dev: 0.00847s window: 11
average rate: 9.948   min: 0.081s max: 0.120s std dev: 0.00614s window: 21

$ curl localhost:8080     -> HTTP 200
battery topic alive: 11593 mV (also confirmed inside each handback poll)
```

Scan at ~10 Hz, battery topic publishing, web UI serving — factory stack fully restored.

## Session battery

Start 11591 mV → end 11593 mV (idle load only; no motion commanded in phase 0).
All takeovers gated ≥11000 mV as designed.

## Incident note (fixed during the run)

First takeover attempt failed its battery read: `set -u` was active while sourcing the
vendor `~/ros2_ws/.robotrc`, which does `export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH`
with `LD_LIBRARY_PATH` unset in a fresh SSH session — the source aborted before reaching the
ROS `local_setup.zsh` lines, so `ros2` was missing from PATH and the read came back empty.
Fix: wrap the source in `set +u` / `set -u` (takeover.sh + handback.sh). The failed attempt
left the factory stack untouched (fail-closed, as designed).
