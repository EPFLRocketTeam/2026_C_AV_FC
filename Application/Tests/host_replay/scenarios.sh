#!/bin/sh
# Run the standard scenarios on decoded test-flight logs.
# Usage: scenarios.sh <IMUSample.csv window> <BaroSample.csv window> [harness]
# Inputs are the decoded SD CSVs (same columns), cut to a window around liftoff
# (e.g. 240 s before to 20 s after) to keep runs fast.
# Times below are for the ERT test flight (onset ~1350.280 s, flown BURN 1350.346 s).
IMU=$1; BARO=$2; HN=${3:-$(dirname "$0")/build/harness}
OUT=$(dirname "$0")/runs; mkdir -p "$OUT"
ONSET=1350280000; TIGN=$((ONSET - 4983000))   # IGNITION entry so that motion = ramp-up end + 50 ms
run() { name=$1; shift; "$HN" imu="$IMU" baro="$BARO" out="$OUT/$name.csv" verbose=1 "$@" > "$OUT/$name.log" 2>&1 & }
run flown            mode=flown t_burn=1350346034
run ign_acchold      mode=ignition t_ign=$TIGN
run ign_acchold_nodet mode=ignition t_ign=$TIGN detector=0
run ign_cable        mode=ignition t_ign=$TIGN t_cable=$((ONSET + 30000))
run reset_in_init    mode=ignition t_ign=$TIGN reset_at=$((ONSET - 20000000))
run reset_in_ign     mode=ignition t_ign=$TIGN reset_at=$((TIGN + 2000000))
run fake_gps         mode=flown t_burn=1350346034 fake_gps=1
for d in 0 100 200 400; do run late_burn_$d mode=flown detector=0 t_burn=$((ONSET + d * 1000)); done
wait
for f in "$OUT"/*.log; do echo "== $(basename $f .log)"; grep -v "GRP-\|^\[ESKF\]" "$f" | grep "t=\|LIFTOFF\]\|APOGEE\|ACC-HOLD\|KAL\]" | grep -v "ARMED\|IMU liftoff DETECTED"; done
cmp -s "$OUT/flown.csv" "$OUT/fake_gps.csv" && echo "fake_gps: output identical to flown (GNSS ignored)"
