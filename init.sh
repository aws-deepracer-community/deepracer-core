#!/bin/bash
git submodule init && git submodule update
sudo apt-get install python3-dev python3-venv
python3 -m venv .env --prompt deepracer
source .env/bin/activate

pip install awscli wheel setuptools pandas
cd dependencies/sagemaker-python-sdk && python setup.py sdist && pip install dist/*.tar.gz && cd ../..