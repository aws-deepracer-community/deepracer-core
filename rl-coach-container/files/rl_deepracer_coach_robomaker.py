#!/usr/bin/env python
# coding: utf-8
from sagemaker.rl import RLEstimator, RLToolkit
import sagemaker
import boto3
import sys
import os
import json
import io

from time import gmtime
sys.path.append("common")


def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


# S3 bucket
boto_session = boto3.session.Session(
    region_name=os.environ.get("AWS_REGION", "us-east-1"))
endpoint_url = os.environ.get("S3_ENDPOINT_URL", None)

if endpoint_url == "":
    s3Client = boto_session.client("s3")
else:
    s3Client = boto_session.client("s3", endpoint_url=endpoint_url)
sage_session = sagemaker.local.LocalSession(
    boto_session=boto_session, s3_endpoint_url=endpoint_url)

# sage_session.default_bucket()
s3_bucket = os.environ.get("MODEL_S3_BUCKET", None)
s3_prefix = os.environ.get("MODEL_S3_PREFIX", None)
pretrained = str2bool(os.environ.get("PRETRAINED", "False"))
s3_pretrained_bucket = os.environ.get("PRETRAINED_S3_BUCKET", "bucket")
s3_pretrained_prefix = os.environ.get(
    "PRETRAINED_S3_PREFIX", "rl-deepracer-pretrained")

# SDK appends the job name and output folder
s3_output_path = 's3://{}/'.format(s3_bucket)

# Hyperparameters
hyperparameter_file = os.environ.get(
    "HYPERPARAMETER_FILE_S3_KEY", "custom_files/hyperparameters.json")

# Model Metadata
modelmetadata_file = os.environ.get(
    "MODELMETADATA_FILE_S3_KEY", "custom_files/model_petadata.json")

# ### Define Variables
# create unique job name
tm = gmtime()
job_name = s3_prefix
s3_prefix_robomaker = job_name + "-robomaker"

# Duration of job in seconds (5 hours)
job_duration_in_seconds = 24 * 60 * 60

aws_region = sage_session.boto_region_name

print("Model checkpoints and other metadata will be stored at: {}{}".format(
    s3_output_path, job_name))

s3_location = "s3://%s/%s" % (s3_bucket, s3_prefix)
print("Uploading to " + s3_location)

# We use the RLEstimator for training RL jobs.
#
# 1. Specify the source directory which has the environment file, preset and training code.
# 2. Specify the entry point as the training code
# 3. Specify the choice of RL toolkit and framework. This automatically resolves to the ECR path for the RL Container.
# 4. Define the training parameters such as the instance count, instance type, job name, s3_bucket and s3_prefix for storing model checkpoints and metadata. **Only 1 training instance is supported for now.**
# 4. Set the RLCOACH_PRESET as "deepracer" for this example.
# 5. Define the metrics definitions that you are interested in capturing in your logs. These can also be visualized in CloudWatch and SageMaker Notebooks.

RLCOACH_PRESET = "deepracer"
sagemaker_image = os.environ.get("SAGEMAKER_IMAGE", "cpu")
# 'local' for cpu, 'local_gpu' for nvidia gpu (and then you don't have to set default runtime to nvidia)
instance_type = "local_gpu" if (sagemaker_image.find("gpu") >= 0) else "local"
image_name = "awsdeepracercommunity/deepracer-sagemaker:{}".format(
    sagemaker_image)

print("Using image %s" % image_name)

# Prepare hyperparameters
hyperparameters_core = {
    "s3_bucket": s3_bucket,
    "s3_prefix": s3_prefix,
    "aws_region": aws_region,
    "model_metadata_s3_key": "s3://{}/{}".format(s3_bucket, modelmetadata_file),
    "RLCOACH_PRESET": RLCOACH_PRESET
}

if pretrained:
    hyperparameters_core['pretrained_s3_bucket'] = "{}".format(
        s3_pretrained_bucket)
    hyperparameters_core['pretrained_s3_prefix'] = s3_pretrained_prefix
    hyperparameters_core['pretrained_checkpoint'] = os.environ.get(
        "PRETRAINED_CHECKPOINT", "best")

max_memory_steps = os.environ.get("MAX_MEMORY_STEPS", "")
if max_memory_steps.isdigit():
    hyperparameters_core['max_memory_steps'] = max_memory_steps

if instance_type == "local_gpu":
    sagemaker_cuda_visible_devices = os.environ.get("CUDA_VISIBLE_DEVICES", "")
    if len(sagemaker_cuda_visible_devices) > 0:
        hyperparameters_core['cuda_visible_devices'] = sagemaker_cuda_visible_devices

# Downloading the hyperparameter file from our local bucket.
hyperparameter_data = io.BytesIO()
s3Client.download_fileobj(
    s3_bucket, hyperparameter_file, hyperparameter_data)
hyperparameters_nn = json.loads(hyperparameter_data.getvalue().decode("utf-8"))
hyperparameters = {**hyperparameters_core, **hyperparameters_nn}
print("Configured following hyperparameters")
print(hyperparameters)
estimator = RLEstimator(entry_point="training_worker.py",
                        source_dir='markov',
                        dependencies=["common/sagemaker_rl", "markov"],
                        sagemaker_session=sage_session,
                        # bypass sagemaker SDK validation of the role
                        role="aaa/",
                        train_instance_type=instance_type,
                        train_instance_count=1,
                        output_path=s3_output_path,
                        base_job_name=job_name,
                        image_name=image_name,
                        train_max_run=job_duration_in_seconds,  # Maximum runtime in seconds
                        hyperparameters=hyperparameters,
                        metric_definitions=RLEstimator.default_metric_definitions(
                            RLToolkit.COACH)
                        )

estimator.fit(job_name=job_name, wait=False)
