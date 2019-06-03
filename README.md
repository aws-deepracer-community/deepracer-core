# deepracer
A repo for running deepracer locally. The rl_coach code comes from https://github.com/awslabs/amazon-sagemaker-examples/tree/master/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo

**I am currently in the process of updating this to the same version that the
Deepracer console uses so things may be broken for a little bit.**

# Running it all through docker
I have been able to improve this process so it's easy for everyone to use. What you will need to run this is:
  - Docker
  - Python3
  - [Minio the S3 emulator](https://min.io/download#/linux)
  - Preferablly a Linux host as Docker works a lot better there
  - A copy of this repo
  
## General notes before we start
You may not need to do all these steps as they pertain to general setup of the host.
- Ensure you have root access to docker through the docker group. See [Post installation steps for docker](https://docs.docker.com/install/linux/linux-postinstall/)
- Please post an issue if you get issues cloning the repo, make sure to use `git clone --recurse-submodules https://github.com/crr0004/deepracer.git` to get them all. You will get an error about benchmarks missing in `sagemaker-tensorflow-container`, you can safely ignore it.
- You may get firewall issues with the docker containers trying to access the minio running outside the sagemaker-local network. You will see errors about *no route to <ip>* from the containers. For fixing this, you can either disable your firewall or allow the docker adapters as trusted adapters.
- More notes to come, if you want anything added here, open an issue please.

## The moving parts in order
- Minio
- Robomaker
- Sagemaker

## Minio
Download the binary from [Minio](https://min.io/download#/linux) and put it somewhere you're okay with having large files.

Then run `source rl_coach\env.sh` to get some reasonable defaults for your environemnt. Then run `./minio server data` to create a folder data. 

**You will need to create a bucket named `bucket` through the web GUI that minio provides, just open http://127.0.0.1:9000 in your browser.**

Then copy the folder *custom_files* into your new bucket as that's where the
defaults expect them to be.

You should source that `env.sh` for every terminal you open when interacting with the deepracer instances because it helps keep everything consistent.

I suggest you `cat rl_coach\env.sh` to see what is being set.

## Sagemaker
I'd suggest you make a python virtual enviornment for this as it will install a fair bit, and with older versions of packages.

To create a virtual environment you can run `python3 -m venv sagemaker_venv` to create the virtual environment in the directory sagemaker_venv. To activate the venv, run `source sagemaker_venv/bin/activate` on linux.

To install sagemaker run `pip install -U sagemaker-python-sdk/ awscli ipython pandas`.

Now you need to get the docker images that sagemaker is expecting. Run `docker pull nabcrr/sagemaker-rl-tensorflow:console`. Now run `docker tag nabcrr/sagemaker-rl-tensorflow:console 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3` to get sagekmaker to use it.

You will need to copy the `config.yaml` file to `~/.sagemaker` to configure
where the temp directories for the sagemaker docker containers are put. I
suggest you edit it to where you want. It is relative to where you run
`rl_deepracer_coach_robomaker.py` from. So make sure to check that folder exists, or change the contents of `~/.sagemaker/config.yaml` to something that does exist. I have it set to a folder a couple directories up.

E.G `mkdir -p ~/.sagemaker && cp config.yaml ~/.sagemaker`.

To set some extra environment variables in Sagemaker SDK, until I figure out a
better way, set the environemnt variable `LOCAL_ENV_VAR_JSON_PATH` to a
`env_vars.json`. E.G export LOCAL_ENV_VAR_JSON_PATH=$(readlink -f ./env_vars.json).

Now you can run `(cd rl_coach; ipython rl_deepracer_coach_robomaker.py)` to start sagemaker.

### Starting robomaker
Firstly to get the images I have built, run `docker pull nabcrr/deepracer_robomaker:console`, no need to alter the tag unless you want to. This image are built from `docker/Robomaker-kinetic-debug.docker`, and the `nabcrr/deepracer_robomaker:1.0b` is built from `docker/Robomaker-kinetic.docker` but shouldn't need to use those docker files unless you want to build it from scratch or do it without docker.

You can run the docker image with `docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -it nabcrr/deepracer_robomaker:console`

If you want an advanced startup that I generally use to see everything you can
use `docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -v $(pwd)/aws-robomaker-sample-application-deepracer/simulation_ws/src:/app/robomaker-deepracer/simulation_ws/src -v $(readlink -f ../robo/checkpoint):/root/.ros/ -it nabcrr/deepracer_robomaker "./run.sh build distributed_training.launch"`. 
This
command mounts all the directories to local directories so you can see all the
files. You can replace the `"./run.sh"` part to `bash` and you will get a
shell in the container.

### Viewing Gazebo and the car running
You can run `vncviewer localhost:8080` to get a VNC view of the running container.

### Altering action space
You now specify your action space in the json file you pass in through
`MODEL_METADATA_FILE_S3_KEY`, which is defaulted to
`bucket/custom_files/model_metadata.json`

# The following is more for your information if you're curious
Some of this information is out of date with the updates to console files. The
same principles still apply though

# How to run and use this - Without the built images
Firstly, this is not for the faint of heart. I am trying to build this repo so it's extremely easy for people to get this running but there are a lot of moving parts and it can be a nightmare. Onward!

## The moving parts
Deepracer is made of the following parts:
- The simulation which is built out of ROS Kinetic, colcon, Gazebo and a simulation environment provided by amazon
- Sagemaker which is built out of the sagemaker SDK and a docker image that is run by it using docker-compose (#2.), which uses image (#1.)
- S3 for some communication and model storage
- Cloudwatch for logs

To get each of these components working in a local environment isn't too hard until you realise the docker images rely on tensorflow with certain CPU features, thus you have to re-build them on your CPU. If you don't want to use ANY AWS services, it means you have to emulate S3 and bypass cloudwatch hits, which is what I have done.

To emulate S3 you can use minio and my patches to various components, which are provided as submodules in this repo. The patches also bypass cloudwatch with environment variables.

# Minio
This service is used to emulate S3 and is very easy to setup and use. Go to the (minio download website)[https://min.io/download] and download your version. Then run the command `minio server .\data` and copy your IP location, key id and access key, you will use these in your enviornment variables.

# Building Robomaker
I have provided a docker build file name Robomarker.docker that does all the build so you can refer to that. In summary, it's install the dependencies of ROS Kinetic and Gazebo. Then install the dependencies of the Deepracer simulation environment. If you want to run those commands outside of a Docker build, I have marked each command that requires sudo.

Run `docker build -t deepracer_robomaker:1.0 -f docker/Robomaker-kinetic-debug.docker`

# Docker images

Following is about building the images used by sagemaker sdk. ~~I will in future provide these in a docker repo somewhere so you don't have to build them.~~

## Building sagemaker-tensorflow-scriptmode
`cd sagemaker-tensorflow-container/docker/1.11.0`
Docker build command `docker build -t 520713654638.dkr.ecr.us-west-2.amazonaws.com/sagemaker-tensorflow-scriptmode:1.11.0-cpu-py3 --build-arg py_version=3 -f Dockerfile.cpu .`

You used to require a copy of tensorflow to build it, you can get one by doing `pip download --no-deps tensorflow==1.11.0`. I have *removed this need by modifying the Dockerfile*.

To get a copy of the `framework_support_installable` it needs, you just need to build the directory using `python3 setup.py sdist`, then move the tar to the docker directory.

## Building sagemaker-containers
You need to build this for sagemaker-rl-tensorflow image as to include the patches for S3 and cloudwatch.

To build it go to sagemaker-containers `cd sagemaker-containers`, and run `python3 setup.py sdist`. Then copy the tar.gz over to sagemaker-rl-container.

## Building sagemaker-rl-tensorflow
`cd sagemaker-rl-container`
Stay at the top level of sagemaker-rl-container repo when building the docker file. Make to sure to build sagemaker-containers before this.

To build the docker image run `docker build -t 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3 --build-arg sagemaker_container=sagemaker_containers-2.4.4.post2.tar.gz --build-arg processor=cpu -f .\coach\docker\0.11.0\Dockerfile.tf .`

If you ever need to quickly rebuild the image with sagemaker-containers, you can run `$(pushd ../sagemaker-containers; python setup.py sdist;popd); $(cp ../sagemaker-containers/dist/*.tar.gz ./); $(docker build -t 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3 --build-arg sagemaker_container=sagemaker_containers-2.4.4.post2.tar.gz --build-arg processor=cpu -f ./coach/docker/0.11.0/Dockerfile.tf .)`.

## Build and install sagemaker sdk
This one is rather easy. Just `cd sagemaker-python-sdk` and run `pip3 install .`, that will install everything it needs for the SDK to run. You will need to have docker and docker-compose in the path of any scripts that invoke the SDK though.

## Building it all example
These commands may work on your system but serve as an example of each step. I am assuming you are in the repo root directory. These were done on a windows machine in powershell. *I suggest you do this on a linux host instead, the following was my first attempt*.

```
$root = $(pwd)
python -m venv venv
./venv/Scripts/Activate.ps1
cd sagemaker-tensorflow-container
python setup.py sdist
cp ./dist/sagemaker_tensorflow_container-2.0.0.tar.gz ./docker/1.11.0/
cd docker/1.11.0/
docker build -t 520713654638.dkr.ecr.us-west-2.amazonaws.com/sagemaker-tensorflow-scriptmode:1.11.0-cpu-py3 --build-arg py_version=3 -f Dockerfile.cpu .
cd $root
cd sagemaker-containers
python setup.py sdist
cp dist/sagemaker_containers-2.4.4.post2.tar.gz ../sagemaker-rl-container
cd $root
cd sagemaker-rl-container
docker build -t 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3 --build-arg sagemaker_container=sagemaker_containers-2.4.4.post2.tar.gz --build-arg processor=cpu -f ./coach/docker/0.11.0/Dockerfile.tf .
cd $root
cd sagemaker-python-sdk
pip install -U .
pip install ipython
pip install -U colorama==0.4
cd $root
docker build -t deepracer_robomaker -f docker/Robomaker.docker .
docker run --name dr deepracer_robomaker
#This comes from minio output
$env:AWS_ACCESS_KEY_ID="PLEXW8P0SOZALM05XQ1A"
$env:AWS_SECRET_ACCESS_KEY="Io0Z7xJOYxqZs3UwkZ7GdVfk7+8cw90roK6QKE0N"
$env:AWS_DEFAULT_REGION="us-east-1"
$env:LOCAL="True"
$env:S3_ENDPOINT_URL=$(write-host "Enter the ip of minio server in this variable instead of copying this")
ipython .\rl_deepracer_coach_robomaker.py


```

# image names
- 1. 520713654638.dkr.ecr.us-west-2.amazonaws.com/sagemaker-tensorflow-scriptmode:1.11.0-cpu-py3
- 2. 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3

The names need to be those as the internals of sagemaker SDK looks for them.

# Command Dump
You can mostly ignore the following, it is for me to dump commands into
```
(cd ~/dev/f/dev/deepracer/; docker run -i -t --name rl_test --rm --env-file deepracer_local/robomaker.env -v $(pwd)/robo/container/:/opt/ml 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3)
docker build -t 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3 --build-arg sagemaker_container=sagemaker_containers-2.4.4.post2.tar.gz --build-arg processor=cpu -f ./Sagemaker-rl.docker .
(cd ../sagemaker-containers/; python setup.py sdist; cp dist/*.tar.gz ../sagemaker-rl-container/)
sudo route add -net 172.17.0.0 gw 10.0.2.2 netmask 255.255.0.0 enp0s3
sudo route del -net 172.17.0.0 netmask 255.255.0.0 enp0s3
cd ~/dev/f/dev/deepracer/deepracer_local
docker run --rm --name dr -e XAUTHORITY=/root/.Xauthority -e DISPLAY_N=:0 --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -v $(pwd)/:/auth/ -it deepracer_robomaker /auth/run.sh
vncviewer localhost:8080
docker build -t deepracer_robomaker -f docker/Robomaker-kinetic-debug.docker .
docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -p 8081:5800 -v $(pwd)/aws-robomaker-sample-application-deepracer/simulation_ws/src:/app/robomaker-deepracer/simulation_ws/src -it deepracer_robomaker "./run.sh build distributed_training.launch"
```
