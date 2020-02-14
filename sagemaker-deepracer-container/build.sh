#!/bin/bash

mkdir -p staging 

cd ../sagemaker-containers/
python setup.py sdist
SAGEMAKER_CONTAINER_DIST=$(ls ./dist/)
cp ./dist/$SAGEMAKER_CONTAINER_DIST ../sagemaker-deepracer-container/staging/
cd ../sagemaker-deepracer-container
cp -r ../amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/lib staging/
cp -r ../amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/markov staging/
cp -r ../amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/rl_coach.patch staging/

for arch in cpu nvidia;
do
    echo docker build -t local/sagemaker-deepracer-container:$arch  . \
        --build-arg sagemaker_containers=$SAGEMAKER_CONTAINER_DIST --build-arg arch=$arch
done