ARG arch
ARG prefix
FROM ${prefix}/sagemaker-tensorflow-container:${arch}

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        jq \
        ffmpeg \
        libjpeg-dev \
        libxrender1 \
        python3.6-dev \
        python3-opengl \
        wget \
        xvfb && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Redis.
RUN cd /tmp && \
    wget http://download.redis.io/redis-stable.tar.gz && \
    tar xvzf redis-stable.tar.gz && \
    cd redis-stable && \
    make && \
    make install

RUN pip install -U --no-cache-dir \
    "PyOpenGL==3.1.0" \
    "pyglet==1.3.2" \
    "gym==0.12.5" \
    "redis>=3.3" \
    "rl-coach-slim==1.0.0"  \
    "urllib3>=1.21.1,<1.26,!=1.25.0,!=1.25.1" \
    "psutil==5.5.0" \
    "botocore<1.16.0,>=1.15.0" \
    retrying \
    eventlet \
    "mxnet-mkl>=1.3.1" \
    "numpy<2.0,>=1.16.0" \
    "sagemaker-containers>=2.7.1" \
    "awscli>=1.18,<2.0" 

COPY ./staging/lib/redis.conf /etc/redis/redis.conf
COPY ./staging/markov /opt/amazon/markov
COPY ./staging/rl_coach.patch /opt/amazon/rl_coach.patch
RUN patch -p1 -N --directory=/usr/local/lib/python3.6/dist-packages/ < /opt/amazon/rl_coach.patch

ENV COACH_BACKEND=tensorflow

# Copy workaround script for incorrect hostname
COPY lib/changehostname.c /
COPY lib/start.sh /usr/local/bin/start.sh
RUN chmod +x /usr/local/bin/start.sh

ENV PYTHONPATH /opt/amazon/:$PYTHONPATH
ENV PATH /opt/ml/code/:$PATH
WORKDIR /opt/ml/code

# Tell sagemaker-containers where the launch point is for training job.
ENV NODE_TYPE SAGEMAKER_TRAINING_WORKER

ENV PYTHONUNBUFFERED 1
# Starts framework
ENTRYPOINT ["bash", "-m", "start.sh", "train"]
