#!/bin/bash
git submodule update --init --recursive
sudo apt-get install python3-dev python3-venv -y
python3 -m venv .env --prompt deepracer
source .env/bin/activate

pip install awscli wheel setuptools pandas
cd dependencies/sagemaker-python-sdk && python setup.py sdist && pip install dist/*.tar.gz && cd ../..

ln -sf dependencies/amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/common/ rl-coach-container/files/common
ln -sf robomaker-container/bundle/sagemaker_rl_agent/lib/python3.5/site-packages/markov/ rl-coach-container/files/markov