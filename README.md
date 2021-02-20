# DeepRacer Core
The DeepRacer Core repository is a utility that is pulling together the different components required for DeepRacer local training. It is not meant for direct usage. If you are looking for the end-user interface to run training locally please go to [Deepracer-for-Cloud](https://github.com/aws-deepracer-community/deepracer-for-cloud)

## Main Components

The primary components of DeepRacer are four docker containers:
* Robomaker Container: Responsible for the robotics environment. Based on ROS + Gazebo as well as the AWS provided "Bundle". Uses components of [AWS Robomaker](https://aws.amazon.com/robomaker/)
* Sagemaker Container: Responsible for training the neural network. Uses components of [AWS Sagemaker](https://aws.amazon.com/robomaker/)
* Reinforcement Learning (RL) Coach: Responsible for preparing and starting the Sagemaker environment.
* Log-Analysis: Providing a containerized Jupyter Notebook for analyzing the logfiles generated. Uses [Deepracer Utils](https://github.com/aws-deepracer-community/deepracer-utils).

## Builds

The built Docker Containers can be found on Docker Hub: 
* https://hub.docker.com/r/awsdeepracercommunity/deepracer-rlcoach
* https://hub.docker.com/r/awsdeepracercommunity/deepracer-robomaker
* https://hub.docker.com/r/awsdeepracercommunity/deepracer-sagemaker
* https://hub.docker.com/r/awsdeepracercommunity/deepracer-analysis
