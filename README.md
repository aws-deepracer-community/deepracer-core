# DeepRacer
A repo for running deepracer locally. The rl_coach code comes from https://github.com/awslabs/amazon-sagemaker-examples/tree/master/reinforcement_learning/rl_deepracer_robomaker_coach_gazebo

**The DeepRacer console undlying bundle can update with no warning so this won't always be up to date with the console.**

**If you can't get this working, please open an issue. It helps with me being able to see issues I might need to fix and it helps everyone else see fixes from issues they might be having. There is a [FAQ in the wiki](https://github.com/crr0004/deepracer/wiki/FAQ) for common issues.**

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

Now you need to get the docker images that sagemaker is expecting. Run `docker pull crr0004/sagemaker-rl-tensorflow:console`. Now run `docker tag crr0004/sagemaker-rl-tensorflow:console 520713654638.dkr.ecr.us-east-1.amazonaws.com/sagemaker-rl-tensorflow:coach0.11-cpu-py3` to get sagekmaker to use it.

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
Firstly to get the images I have built, run `docker pull crr0004/deepracer_robomaker:console`, no need to alter the tag unless you want to. This image are built from `docker/Robomaker-kinetic-debug.docker`, and the `crr0004/deepracer_robomaker:1.0b` is built from `docker/Robomaker-kinetic.docker` but shouldn't need to use those docker files unless you want to build it from scratch or do it without docker.

You can run the docker image with `docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -it crr0004/deepracer_robomaker:console`

If you want an advanced startup that I generally use to see everything you can
use `docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -p 8080:5900 -v $(pwd)/aws-robomaker-sample-application-deepracer/simulation_ws/src:/app/robomaker-deepracer/simulation_ws/src -v $(readlink -f ../robo/checkpoint):/root/.ros/ -it crr0004/deepracer_robomaker "./run.sh build distributed_training.launch"`. 
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

# [FAQ](https://github.com/crr0004/deepracer/wiki/FAQ)

## Not listed here? Look at the closed/open issues or open a new one if you are not sure


---
