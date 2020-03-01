#!/usr/bin/env python

"""script to download yaml param from S3 bucket to local directory and start training/eval ROS launch

Example:
    $ python download_params_and_roslaunch_agent.py s3_region, s3_bucket, s3_prefix, s3_yaml_name, launch_name
"""

# import argparse
import botocore
import boto3
import json
import logging
import rospy
import os
import sys
import yaml

from markov import utils_parse_model_metadata
from markov.architecture.constants import Input
from markov.utils import Logger
from markov.utils import DEFAULT_COLOR, SIMAPP_MEMORY_BACKEND_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500
from subprocess import Popen

logger = Logger(__name__, logging.INFO).get_logger()

def main():
    # parse arguement
    s3_region = sys.argv[1]
    s3_bucket = sys.argv[2]
    s3_prefix = sys.argv[3]
    s3_yaml_name = sys.argv[4]
    launch_name = sys.argv[5]
    # yaml_key = os.path.normpath(os.path.join(s3_prefix, s3_yaml_name))
    yaml_key = os.path.normpath('custom_files/training_params.yaml')
    json_key = os.environ.get('MODEL_METADATA_FILE_S3_KEY', os.path.join(s3_prefix, 'model/model_metadata.json'))
    json_key = json_key.replace('s3://{}/'.format(s3_bucket), '')
    local_yaml_path = os.path.abspath(os.path.join(os.getcwd(), s3_yaml_name))
    local_model_metadata_path = os.path.abspath(os.path.join(os.getcwd(), 'model_metadata.json'))

    # create boto3 session/client and download yaml/json file
    session = boto3.session.Session()
    s3_endpoint_url = os.environ.get("S3_ENDPOINT_URL")
    s3_client = session.client('s3', region_name=s3_region, endpoint_url=s3_endpoint_url)
    s3_client.download_file(Bucket=s3_bucket, Key=yaml_key, Filename=local_yaml_path)
    s3_client.download_file(Bucket=s3_bucket, Key=json_key, Filename=local_model_metadata_path)

    # get sensor and network topology
    sensors, network, simapp_version = utils_parse_model_metadata.parse_model_metadata(local_model_metadata_path)
    car_color = get_car_color(local_yaml_path)

    # launch the rollout_rl_agent.launch for trainig or evaluation_rl_agent.launch for evaluation
    cmd = [
        ''.join(("roslaunch deepracer_simulation {} ".format(launch_name),
                 "local_yaml_path:={} include_second_camera:={} ".format(local_yaml_path,
                                                                         Input.STEREO.value in sensors),
                 "include_lidar_sensor:={} network:={} ".format(Input.LIDAR.value in sensors, network),
                 "car_color:={} simapp_version:={}".format(car_color, simapp_version)))
    ]
    Popen(cmd, shell=True, executable="/bin/bash")

def get_car_color(local_yaml_path):
    with open(local_yaml_path, 'r') as stream:
        try:
            yaml_dict = yaml.safe_load(stream)
            return yaml_dict.get("CAR_COLOR", DEFAULT_COLOR)
        except yaml.YAMLError as exc:
            logger.info("yaml read error: {}".format(exc))

if __name__ == '__main__':
    rospy.init_node('download_params_and_roslaunch_agent_node', anonymous=True)
    main()
