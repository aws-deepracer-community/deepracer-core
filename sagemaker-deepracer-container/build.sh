#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
        echo "Requested to stop."
        exit 1
}
PREFIX="local"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR/sagemaker-tensorflow-container/
python setup.py sdist
cp dist/*.tar.gz docker/build_artifacts/
git apply $DIR/lib/dockerfile-1.15.2-cudnn.patch
cd docker/build_artifacts

for arch in cpu gpu; do
    docker build . -t $PREFIX/sagemaker-tensorflow-container:$arch -f ../1.15.2/py3/Dockerfile.$arch --build-arg py_version=3
done
rm *.tar.gz

cd $DIR/sagemaker-tensorflow-container/
git apply --reverse ../lib/dockerfile-1.15.2-cudnn.patch

## Second stage
cd $DIR
rm -rf $DIR/staging
mkdir -p $DIR/staging 
cp -r ../dependencies/amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/lib staging/
cp -r ../dependencies/amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/markov staging/
cp -r ../dependencies/amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/rl_coach.patch staging/

for arch in cpu gpu;
do
    docker build -t $PREFIX/sagemaker-deepracer-container:$arch . --build-arg arch=$arch --build-arg prefix=$PREFIX
done