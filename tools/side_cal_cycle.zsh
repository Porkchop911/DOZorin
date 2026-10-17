source ~/hwr/deploy/env.zsh
export HWR_GATEWAY_TOKEN=$(grep -o 'TOKEN=.*' ~/hwr/config/hwr.env | head -1 | cut -d= -f2)
H="Authorization: Bearer $HWR_GATEWAY_TOKEN"
curl -s -m 5 -o /dev/null -X POST -H "$H" localhost:8090/estop/reset
cd ~/hwr/ros2_ws/src/hwr_agent
python3 - <<'PY'
import json, math, time
from hwr_agent.skills import GatewaySkills
s = GatewaySkills(max_actions=50, timeout=300)
p = s.pick("blue")
row = {"time": time.time(), "pick": p}
if p.get("outcome") == "holding":
    row["place"] = s.place(on="red", level=1, color="blue", pitch_deg=p.get("pitch_deg"))
    sp = row["place"].get("spot") or {}
    if sp:
        row["pan_deg"] = round(math.degrees(math.atan2(sp["y"], sp["x"] - 0.0466)), 1)
s.arm_pose("forward")
open("/home/ubuntu/hwr/state/side_cal.jsonl", "a").write(json.dumps(row) + "\n")
pl = row.get("place", {})
print("pick", p.get("outcome"), p.get("pitch_deg"), "| place", pl.get("outcome"), pl.get("pitch_deg"),
      "err(radial,side)", pl.get("error_radial_side"), "pan", row.get("pan_deg"), "spot", pl.get("spot"))
PY
