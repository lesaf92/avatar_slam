#!/bin/bash
# Smoke test of DAVE's multibeam sonar in the avatar-dave container (task T-S2-05, LOG L34):
# a paused world is stepped three times by 110 ms (one 10 Hz sonar frame each), twice, and
# the frames are checked against the geometry and against each other.
#   AVATAR_TIER2_IMAGE=avatar-dave docker/tier2.sh bash experiments/gazebo/sonar_smoke/run.sh
set -u
HERE=/work/experiments/gazebo/sonar_smoke
OUT=/work/results/sonar_smoke
mkdir -p "$OUT"
# The sonar plugin creates an (empty) debug_timings.txt in the working directory.
cd "$OUT"
for run in a b; do
  gz sim -s --headless-rendering -v 1 "$HERE/world.sdf" > "$OUT/gz_$run.log" 2>&1 &
  GZ=$!
  sleep 25
  /usr/bin/python3 "$HERE/listen_raw.py" 60 "$OUT/raw_$run.npz" 3 > "$OUT/listen_$run.log" 2>&1 &
  L=$!
  sleep 5
  for i in 1 2 3; do
    gz service -s /world/sonar_smoke/control --reqtype gz.msgs.WorldControl \
      --reptype gz.msgs.Boolean --timeout 5000 --req 'multi_step: 110' > /dev/null
    sleep 4
  done
  wait $L
  kill $GZ 2>/dev/null
  sleep 2
done
/opt/venv/bin/python "$HERE/check.py" "$OUT/raw_a.npz" "$OUT/raw_b.npz"
