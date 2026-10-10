#!/bin/sh
# The Tier-2 recorder in stream mode (T-S3-03): Gazebo on <dir>/world.sdf, then
# `gz_recorder --stream <dir>/plan.txt <timeout_s>` with this script's stdin and stdout, for the
# ROS 2 node avatar_sim/gazebo_rigs. Run in the avatar-tier2 image:
#   docker run -i ... avatar-tier2 sh experiments/gazebo/stream.sh <dir> [timeout_s]
set -eu
dir=$1
timeout=${2:-10}
export GZ_IP="${GZ_IP:-127.0.0.1}"
gz sim -s --headless-rendering -v 2 "$dir/world.sdf" > "$dir/gz.log" 2>&1 &
experiments/gazebo/build/gz_recorder --stream "$dir/plan.txt" "$timeout"
