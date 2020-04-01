#!/bin/bash
git submodule update --init --recursive
sudo apt-get install python3-dev python3-venv -y
python3 -m venv .env --prompt deepracer
source .env/bin/activate

mkdir -p config rl-coach-container/staging

pip install awscli wheel setuptools pandas sagemaker
# cd dependencies/sagemaker-python-sdk && python setup.py sdist && pip install dist/*.tar.gz && cd ../..

cp -R dependencies/amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/common/ rl-coach-container/files/common
cp -R robomaker-container/bundle/sagemaker_rl_agent/lib/python3.5/site-packages/markov/ rl-coach-container/files/markov
# cp dependencies/sagemaker-python-sdk/dist/*.tar.gz rl-coach-container/staging
