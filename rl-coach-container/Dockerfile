FROM ubuntu:20.04

# install docker
RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    gnupg-agent \
    software-properties-common \
    python3 \
    python3-pip
RUN curl -fsSL https://download.docker.com/linux/debian/gpg | apt-key add - && \
	 add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
RUN apt-get update && apt-get install --no-install-recommends -y docker-ce-cli && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose && chmod +x /usr/local/bin/docker-compose

# create sagemaker configuration
RUN mkdir -p /root/.sagemaker /robo/container 
COPY rl-coach-container/files/config.yaml /root/.sagemaker/config.yaml

# add required deepracer directories to the container
# RUN mkdir -p /deepracer && mkdir -p /deepracer/rl_coach && mkdir -p /deepracer/sagemaker-python-sdk
WORKDIR /deepracer

# install dependencies
RUN pip install --no-cache-dir -U "sagemaker<2" "protobuf<3.20" awscli ipython pandas "urllib3<1.27,>=1.25.4" "pyyaml==3.13" "python-dateutil==2.8.0"

ADD rl-coach-container/files/common /deepracer/common
ADD robomaker-container/bundle/markov /deepracer/markov
COPY rl-coach-container/files/rl_deepracer_coach_robomaker.py /deepracer/

# set command
CMD (python3 rl_deepracer_coach_robomaker.py)

# Versioning
ARG IMG_VERSION
LABEL maintainer "AWS DeepRacer Community - deepracing.io"
LABEL version $IMG_VERSION