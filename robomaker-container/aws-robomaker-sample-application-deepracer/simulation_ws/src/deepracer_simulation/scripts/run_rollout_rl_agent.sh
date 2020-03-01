#!/usr/bin/env bash

set -ex

#export PYTHONPATH=${COLCON_BUNDLE_PREFIX}/usr/local/lib/python3.5/dist-packages/:$PYTHONPATH
export PYTHONUNBUFFERED=1

if ! python3 -m markov.rollout_worker; then
    exit
fi