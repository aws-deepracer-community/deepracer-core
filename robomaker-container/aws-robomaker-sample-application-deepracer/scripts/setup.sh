#!/usr/bin/env bash

set -ex

apt-get update
source /opt/ros/$ROS_DISTRO/setup.sh
rosdep update

apt-get install -y python3-apt python3-pip
pip3 install -U setuptools
pip3 install -U colcon-common-extensions colcon-ros-bundle

rosdep install --from-paths ./simulation_ws/src --ignore-src -r -y
