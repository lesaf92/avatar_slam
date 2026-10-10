#!/bin/sh
# The DAVE sonar pass in stream mode (T-S3-04): Gazebo on <dir>/world.sdf (the acoustic world),
# then `sonar_driver.py --stream <dir>/plan.json` with this script's stdin and stdout, for the
# ROS 2 node avatar_sim/gazebo_sonar. Run in the avatar-dave image:
#   docker run -i ... avatar-dave sh experiments/gazebo/sonar_stream.sh <dir>
set -eu
dir=$1
export GZ_IP="${GZ_IP:-127.0.0.1}"
# DAVE's sonar topics stay inside this container (one ROS graph per run, on localhost)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export HOME="${HOME:-/tmp}"
# DAVE's sonar plugin writes debug_timings.txt in its working directory
(cd "$dir" && exec gz sim -s --headless-rendering -v 2 world.sdf > gz.log 2>&1) &
/usr/bin/python3 experiments/gazebo/sonar_driver.py --stream "$dir/plan.json"
