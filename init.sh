#!/bin/bash
sudo apt-get install python3-dev python3-venv
python3 -m venv .env --prompt deepracer
source .env/bin/activate

pip install awscli sagemaker wheel setuptools pandas