FROM ubuntu:18.04
LABEL maintainer "Lars Ludvigsen <lars@ludvig.no>"

# install docker
RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    gnupg-agent \
    software-properties-common \
    python3 python3-pip python3-setuptools python3-dev \
    build-essential
RUN curl -fsSL https://download.docker.com/linux/debian/gpg | apt-key add - && \
	 add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
RUN apt-get update && apt-get install --no-install-recommends -y docker-ce-cli && apt-get clean && rm -rf /var/lib/apt/lists/*

# create sagemaker configuration
RUN mkdir -p /root/.sagemaker /robo/container 
COPY files/config.yaml /root/.sagemaker/config.yaml
COPY staging/sagemaker* .

# install dependencies
RUN pip3 install --no-cache-dir -U sagemaker*.tar.gz awscli ipython pandas "rl-coach-slim==1.0.0" "urllib3==1.22" "pyyaml==3.13" "python-dateutil==2.8.0"

# add required deepracer directories to the container
# RUN mkdir -p /deepracer && mkdir -p /deepracer/rl_coach && mkdir -p /deepracer/sagemaker-python-sdk
WORKDIR /deepracer
COPY files/rl_deepracer_coach_robomaker.py /deepracer/
ADD staging/markov markov
ADD staging/common common

# set command
RUN pwd && ls -l
CMD (ipython rl_deepracer_coach_robomaker.py)
