#!/usr/bin/env bash

COLCON_LOG_PATH="$@/logs" colcon build --base-paths "$@" --build-base "$@/build" --install-base "$@/install"
