# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup, find_packages

# Package meta-data.
NAME = 'sagemaker_rl_agent'
REQUIRES_PYTHON = '>=3.5.0'

setup(
    name=NAME,
	zip_safe=False,
    version='0.0.1',
    packages=find_packages(),
    python_requires=REQUIRES_PYTHON,
    install_requires=[
        'annoy==1.8.3',
        'Pillow==4.3.0',
        'matplotlib==2.0.2',
        'numpy==1.14.5',
        'pandas==0.22.0',
        'pygame==1.9.3',
        'PyOpenGL==3.1.0',
        'scipy==1.2.1',
        'scikit-image==0.15.0',
        'futures==3.1.1',
        'boto3==1.9.23',
        'minio==4.0.5',
        #'kubernetes==7.0.0',
        'rl-coach-slim==1.0.0',
        #'PyYAML==3.13',
        'rospkg==1.1.7',
        'shapely==1.6.4',
        'tensorflow==1.11',
        'redis==3.2.1',
        'opencv-python==4.1.1.26',
    ],
    entry_points = {
        'console_scripts': [
            'run_rollout_rl_agent=markov.rollout_worker:main',
            'run_local_rl_agent=envs.local_worker:main'
        ],
    }
)
