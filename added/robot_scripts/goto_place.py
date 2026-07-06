#!/usr/bin/env python3
"""goto_place.py <name> [--list] — drive to a named waypoint from ~/places.yaml via goto.py."""
import subprocess, sys, yaml, os
P = os.path.expanduser("~/places.yaml")
places = yaml.safe_load(open(P)) or {}
if len(sys.argv) < 2 or sys.argv[1] in ("--list", "-l"):
    for k, v in places.items():
        print(f"{k}: x={v['x']} y={v['y']} yaw={v.get('yaw_deg', 0)}")
    sys.exit(0)
name = sys.argv[1].strip().lower()
if name not in places:
    # tolerate simple aliases like "the kitchen"
    cand = [k for k in places if k in name or name in k]
    if len(cand) == 1:
        name = cand[0]
    else:
        print(f"unknown place '{name}'. known: {', '.join(places)}"); sys.exit(1)
p = places[name]
print(f"-> {name}: x={p['x']} y={p['y']} yaw={p.get('yaw_deg', 0)}")
sys.exit(subprocess.call(["python3", os.path.expanduser("~/goto.py"),
                          str(p["x"]), str(p["y"]), str(p.get("yaw_deg", 0))]))
