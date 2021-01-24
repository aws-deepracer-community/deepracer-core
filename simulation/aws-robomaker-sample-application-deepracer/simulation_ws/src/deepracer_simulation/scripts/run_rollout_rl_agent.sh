#!/usr/bin/env bash

set -ex

#export PYTHONPATH=${COLCON_BUNDLE_PREFIX}/usr/local/lib/python3.5/dist-packages/:$PYTHONPATH
export PYTHONUNBUFFERED=1

python3 -m markov.rollout_worker
