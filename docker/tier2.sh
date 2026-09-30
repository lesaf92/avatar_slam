#!/bin/sh
# Run a command in the Tier-2 recording container (docker/Dockerfile.tier2) on the GPU.
#   docker/tier2.sh python experiments/gazebo/record.py --seed 0 --out results/tier2/harbor_fleet_seed0
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec docker run --rm --gpus all --user "$(id -u):$(id -g)" \
    -v "$ROOT":/work -w /work "${AVATAR_TIER2_IMAGE:-avatar-tier2}" "$@"
