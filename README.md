# DeepRacer
A repo for running deepracer locally. The rl_coach code comes from https://github.com/awslabs/amazon-sagemaker-examples/tree/master/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo

**The DeepRacer console undlying bundle can update with no warning so this won't always be up to date with the console.**

**If you can't get this working, please open an issue. It helps with me being able to see issues I might need to fix and it helps everyone else see fixes from issues they might be having. There is a [FAQ in the wiki](https://github.com/crr0004/deepracer/wiki/FAQ) for common issues.**

For additonal help with OSX setup, please [refer to a supplimental guide provided by joezen777](https://gist.github.com/joezen777/6657bbe2bd4add5d1cdbd44db9761edb) in [issue #11](https://github.com/crr0004/deepracer/issues/11).

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

To install sagemaker run `pip install -U sagemaker-python-sdk/ awscli pandas`.

Now you need to get the docker images that sagemaker is expecting. Run `docker pull crr0004/sagemaker-rl-tensorflow:console`. I have fixed the python script so it uses this image directly now, no more tagging needed.

You will need to copy the `config.yaml` file to `~/.sagemaker` to configure
where the temp directories for the sagemaker docker containers are put. I
suggest you edit it to where you want. It is relative to where you run
`rl_deepracer_coach_robomaker.py` from. So make sure to check that folder exists, or change the contents of `~/.sagemaker/config.yaml` to something that does exist. I have it set to a folder a couple directories up.

E.G `mkdir -p ~/.sagemaker && cp config.yaml ~/.sagemaker`.

To set some extra environment variables in Sagemaker SDK, until I figure out a
better way, set the environemnt variable `LOCAL_ENV_VAR_JSON_PATH` to a
`env_vars.json`. E.G export LOCAL_ENV_VAR_JSON_PATH=$(readlink -f ./env_vars.json).

Now you can run `(cd rl_coach; python rl_deepracer_coach_robomaker.py)` to start sagemaker.

### GPU Acceleration
You can change the image name in `rl_deepracer_coach_robomaker.py` to your respective GPU type and do the setup needed for each type, see each section.

### NVIDIA GPU Acceleration
You can change the image name in `rl_deepracer_coach_robomaker.py` to "crr0004/sagemaker-rl-tensorflow:nvidia" to use GPU accerlation. You will also need to setup docker to use the GPU by following https://github.com/NVIDIA/nvidia-docker.

In this file update `instance_type` to `local_gpu` to run sagemaker in nvidia runtime.

You can also uncommment the line in `env.sh` that is `#export
LOCAL_EXTRA_DOCKER_COMPOSE_PATH=$(readlink -f ./docker_compose_extra.json)` to
cause privilaged to be passed to the docker compose command. You can also use
the file `docker_compose_extra.json` to modify the docker compose file that is
used to launch the sagemaker container.

### AMD GPU Acceleration
You can change the image name in `rl_deepracer_coach_robomaker.py` to  "crr0004/sagemaker-rl-tensorflow:amd". 
You will need to install [ROCm](https://github.com/RadeonOpenCompute/ROCm) and then ensure there is the kfd device on your system. 
If you're using an upstream kernel, there is a section in the [ROCm Readme](https://github.com/RadeonOpenCompute/ROCm#using-rocm-with-upstream-kernel-drivers) to enable a kfd device.
Then you will need to uncomment the line in `env.sh` to to enable `LOCAL_EXTRA_DOCKER_COMPOSE` and add 
```
"devices": [
	"/dev/kfd:/dev/kfd",
	"/dev/dri:/dev/dri"
]
```
to `docker_compose_extra.json`

### Starting robomaker
Firstly to get the images I have built, run `docker pull crr0004/deepracer_robomaker:console`, no need to alter the tag unless you want to. This image are built from `docker/Robomaker-kinetic-debug.docker`, and the `crr0004/deepracer_robomaker:1.0b` is built from `docker/Robomaker-kinetic.docker` but shouldn't need to use those docker files unless you want to build it from scratch or do it without docker.

You will need to alter the `robomaker.env` file to change the `WORLD_NAME` to the track you want, and anything else.

You can run the docker image with `docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -it crr0004/deepracer_robomaker:console`

If you want an advanced startup that I generally use to see everything you can
use `docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -v $(pwd)/simulation/aws-robomaker-sample-application-deepracer/simulation_ws/src:/app/robomaker-deepracer/simulation_ws/src -v $(readlink -f ../robo/checkpoint):/root/.ros/ -it crr0004/deepracer_robomaker:console "./run.sh build distributed_training.launch"`. 
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

### dr_util.py
*WARNING*
THIS SCRIPT MODIFIES FILES IN YOUR AWS S3 BUCKET (DELETES AND UPLOADS).
WHILE THE AUTHORS HAVE TAKEN CARE TO NOT MAKE IT HARMFUL,
THEY TAKE NO RESPONSIBILITY FOR ANY DAMAGES IT MAY CAUSE, ESPECIALLY (BUT NOT ONLY) IF MISCONFIGURED.
USE AT YOUR OWN RISK.

This script's aim is to make it easier to create snapshots of models to upload for submission.
It has been written to work in sagemaker_venv so make sure to activate it before using.
It also assumes that you have awscli installed and configured with access to DeepRacer's S3 bucket.

To use it call it first: `./dr_util.py init`, then set values in created `dr_util_config.json`. It has been added to .gitignore.

To learn more about usage, run `./dr_util.py -h`

# [FAQ](https://github.com/crr0004/deepracer/wiki/FAQ)
# [Wiki](https://github.com/crr0004/deepracer/wiki)

## Not listed here? Look at the closed/open issues or open a new one if you are not sure


---
