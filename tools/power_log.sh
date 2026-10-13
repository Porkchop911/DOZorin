#!/bin/bash
# Logs power + thermal readings every ${1:-30} s to ~/hwr/state/power.log (TSV). Runs on the robot.
# Columns: time, battery_V, VDD_IN_mW, CPU_GPU_CV_mW, SOC_mW, tj_C, cpu_C, gpu_C, fan_pwm, oc_alarms
set -u
interval=${1:-30}
log=$HOME/hwr/state/power.log
token=$(sed -n 's/^HWR_GATEWAY_TOKEN=//p' $HOME/hwr/config/hwr.env)
[[ -s $log ]] || printf 'time\tbattery_V\tVDD_IN_mW\tCPU_GPU_CV_mW\tSOC_mW\ttj_C\tcpu_C\tgpu_C\tfan_pwm\toc_alarms\n' > $log
temp() { local z; for z in /sys/class/thermal/thermal_zone*; do [[ $(<$z/type) == "$1" ]] && awk '{printf "%.1f", $1/1000}' $z/temp 2>/dev/null; done; }
while true; do
    batt=$(curl -s -m 3 -H "Authorization: Bearer $token" http://127.0.0.1:8090/status |
           python3 -c 'import json,sys; d=json.load(sys.stdin); print(round(d["battery_voltage"],3) if d.get("battery_fresh") else "stale")' 2>/dev/null || echo na)
    rails=$(timeout 3 tegrastats --interval 1000 | head -1)
    rail() { sed -n "s/.*$1 \([0-9]*\)mW.*/\1/p" <<<"$rails"; }
    fan=$(cat /sys/class/hwmon/hwmon0/pwm1 2>/dev/null || echo na)
    oc=$(cat /sys/class/hwmon/hwmon3/*alarm 2>/dev/null | paste -sd, -)
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$(date '+%F %T')" "$batt" "$(rail VDD_IN)" \
        "$(rail VDD_CPU_GPU_CV)" "$(rail VDD_SOC)" "$(temp tj-thermal)" "$(temp cpu-thermal)" \
        "$(temp gpu-thermal)" "$fan" "${oc:-na}" >> $log
    sleep $interval
done
