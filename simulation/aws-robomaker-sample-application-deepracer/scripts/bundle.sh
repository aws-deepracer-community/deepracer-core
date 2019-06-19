#!/usr/bin/env bash

COLCON_LOG_PATH="$@/logs" colcon bundle --base-paths "$@" --build-base "$@/build" --install-base "$@/install" --bundle-base "$@/bundle"
