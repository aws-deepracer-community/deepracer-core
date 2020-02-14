#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
        echo "Requested to stop."
        exit 1
}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR/sagemaker-tensorflow-container/
python setup.py sdist
cp dist/*.tar.gz docker/build_artifacts/
git apply $DIR/lib/dockerfile-cudnn.patch
cd docker/build_artifacts

for arch in cpu gpu; do
    docker build . -t local/sagemaker-tensorflow-container:$arch -f ../1.15.0/py3/Dockerfile.$arch --build-arg py_version=3
done
rm *.tar.gz

cd $DIR/sagemaker-tensorflow-container/
git apply --reverse ../lib/dockerfile-cudnn.patch

## Second stage
rm -rf $DIR/staging
mkdir -p $DIR/staging 
cd $DIR/sagemaker-containers/
python setup.py sdist
SAGEMAKER_CONTAINER_DIST=$(ls ./dist/)
cp ./dist/$SAGEMAKER_CONTAINER_DIST $DIR/staging/

cd $DIR
cp -r ../amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/lib staging/
cp -r ../amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/markov staging/
cp -r ../amazon-sagemaker-examples/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo/src/rl_coach.patch staging/

for arch in cpu gpu;
do
    docker build -t local/sagemaker-deepracer-container:$arch . --build-arg sagemaker_containers=$SAGEMAKER_CONTAINER_DIST --build-arg arch=$arch
done