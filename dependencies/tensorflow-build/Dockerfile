FROM ubuntu:18.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    ca-certificates \
    curl \
    wget \
    python3-dev \ 
    python3-pip \
    apt-transport-https \
    gnupg \
    openjdk-11-jdk \
    unzip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U 'pip<20' 'numpy==1.18.5' setuptools wheel
RUN pip3 install -U keras_preprocessing keras_applications

ARG bazel_path=https://github.com/bazelbuild/bazel/releases/download/0.15.2/
ARG bazel_file=bazel-0.15.2-installer-linux-x86_64.sh 

RUN wget $bazel_path$bazel_file && bash $bazel_file && rm $bazel_file

# Set environment variables for MKL
# For more about MKL with TensorFlow see:
# https://www.tensorflow.org/performance/performance_guide#tensorflow_with_intel%C2%AE_mkl_dnn
ENV KMP_AFFINITY=granularity=fine,compact,1,0 KMP_BLOCKTIME=1 KMP_SETTINGS=0
ADD icu-signature.diff /root
ADD build.sh /root

RUN ln -s /usr/bin/python3 /usr/bin/python
RUN mkdir /tensorflow_src /out
WORKDIR /tensorflow_src

RUN git clone --single-branch --branch r1.12 https://github.com/tensorflow/tensorflow.git /tensorflow_src
RUN patch -p 1 < /root/icu-signature.diff
RUN ./configure
ADD tf_configure.bazelrc .tf_configure.bazelrc

ENTRYPOINT "/root/build.sh"
