#!/bin/bash
git submodule update --init --recursive
sudo apt-get install python3-dev python3-venv -y
python3 -m venv .env --prompt deepracer
source .env/bin/activate

pip install awscli wheel setuptools pandas sagemaker
