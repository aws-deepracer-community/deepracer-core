"""
this rollout worker:

- restores a model from disk
- evaluates a predefined number of episodes
- contributes them to a distributed memory
- exits
"""

import argparse
import json
import math
import os
import random
import sys
import time

from google.protobuf import text_format
from tensorflow.python.training.checkpoint_state_pb2 import CheckpointState

import markov
from markov.s3_boto_data_store import S3BotoDataStore, S3BotoDataStoreParameters
from markov.s3_client import SageS3Client
from markov.utils import register_custom_environments, load_model_metadata
from rl_coach.base_parameters import TaskParameters, DistributedCoachSynchronizationType
from rl_coach.core_types import RunPhase, EnvironmentEpisodes, EnvironmentSteps
from rl_coach.data_stores.data_store import DataStoreParameters, SyncFiles
from rl_coach.logger import screen
from rl_coach.memories.backend.redis import RedisPubSubMemoryBackendParameters
from rl_coach.utils import short_dynamic_import

CUSTOM_FILES_PATH = "./custom_files"
if not os.path.exists(CUSTOM_FILES_PATH):
    os.makedirs(CUSTOM_FILES_PATH)


# Q: specify alternative distributed memory, or should this go in the preset?
# A: preset must define distributed memory to be used. we aren't going to take
#    a non-distributed preset and automatically distribute it.

def has_checkpoint(checkpoint_dir):
    """
    True if a checkpoint is present in checkpoint_dir
    """
    if os.path.isdir(checkpoint_dir):
        if len(os.listdir(checkpoint_dir)) > 0:
            return os.path.isfile(os.path.join(checkpoint_dir, "checkpoint"))

    return False


def wait_for_checkpoint(checkpoint_dir, data_store=None, timeout=10):
    """
    block until there is a checkpoint in checkpoint_dir
    """
    for i in range(timeout):
        if data_store:
            data_store.load_from_store()

        if has_checkpoint(checkpoint_dir):
            return
        time.sleep(10)

    # one last time
    if has_checkpoint(checkpoint_dir):
        return

    raise ValueError((
        'Waited {timeout} seconds, but checkpoint never found in '
        '{checkpoint_dir}'
    ).format(
        timeout=timeout,
        checkpoint_dir=checkpoint_dir,
    ))


def get_latest_checkpoint(checkpoint_dir):
    if os.path.exists(os.path.join(checkpoint_dir, 'checkpoint')):
        ckpt = CheckpointState()
        contents = open(os.path.join(checkpoint_dir, 'checkpoint'), 'r').read()
        text_format.Merge(contents, ckpt)
        # rel_path = os.path.relpath(ckpt.model_checkpoint_path, checkpoint_dir)
        rel_path = ckpt.model_checkpoint_path
        return int(rel_path.split('_Step')[0])


def download_customer_reward_function(s3_client, reward_file_s3_key):
    reward_function_local_path = os.path.join(CUSTOM_FILES_PATH, "customer_reward_function.py")
    success_reward_function_download = s3_client.download_file(s3_key=reward_file_s3_key,
                                                               local_path=reward_function_local_path)
    if not success_reward_function_download:
        print("Could not download the customer reward function file. Job failed!")
        sys.exit(1)


def download_custom_files_if_present(s3_client, s3_prefix):
    environment_file_s3_key = os.path.normpath(s3_prefix + "/environments/deepracer_racetrack_env.py")
    environment_local_path = os.path.join(CUSTOM_FILES_PATH, "deepracer_racetrack_env.py")
    success_environment_download = s3_client.download_file(s3_key=environment_file_s3_key,
                                                           local_path=environment_local_path)

    preset_file_s3_key = os.path.normpath(s3_prefix + "/presets/preset.py")
    preset_local_path = os.path.join(CUSTOM_FILES_PATH, "preset.py")
    success_preset_download = s3_client.download_file(s3_key=preset_file_s3_key,
                                                      local_path=preset_local_path)
    return success_preset_download, success_environment_download


def should_stop(checkpoint_dir):
    if os.path.exists(os.path.join(checkpoint_dir, SyncFiles.FINISHED.value)):
        print("Received termination signal from trainer. Goodbye.")
        return True
    else:
        return False


def rollout_worker(graph_manager, checkpoint_dir, data_store, num_workers):
    """
    wait for first checkpoint then perform rollouts using the model
    """

    wait_for_checkpoint(checkpoint_dir, data_store)

    task_parameters = TaskParameters()
    task_parameters.__dict__['checkpoint_restore_dir'] = checkpoint_dir

    graph_manager.create_graph(task_parameters)
    with graph_manager.phase_context(RunPhase.TRAIN):
        last_checkpoint = 0
        act_steps = math.ceil(
            (graph_manager.agent_params.algorithm.num_consecutive_playing_steps.num_steps) / num_workers)

        for i in range(math.ceil(graph_manager.improve_steps.num_steps / act_steps)):

            if should_stop(checkpoint_dir):
                break

            if type(graph_manager.agent_params.algorithm.num_consecutive_playing_steps) == EnvironmentSteps:
                graph_manager.act(EnvironmentSteps(num_steps=act_steps + random.randint(1,3)),
                                  wait_for_full_episodes=graph_manager.agent_params.algorithm.act_for_full_episodes)
            elif type(graph_manager.agent_params.algorithm.num_consecutive_playing_steps) == EnvironmentEpisodes:
                graph_manager.act(EnvironmentEpisodes(num_steps=act_steps + random.randint(1,3)))

            if graph_manager.agent_params.algorithm.distributed_coach_synchronization_type == DistributedCoachSynchronizationType.SYNC:
                data_store.load_from_store(expected_checkpoint_number=last_checkpoint + 1)
                if should_stop(checkpoint_dir):
                    break
                last_checkpoint = get_latest_checkpoint(checkpoint_dir)
                graph_manager.restore_checkpoint()

            if graph_manager.agent_params.algorithm.distributed_coach_synchronization_type == DistributedCoachSynchronizationType.ASYNC:
                new_checkpoint = get_latest_checkpoint(checkpoint_dir)
                if new_checkpoint > last_checkpoint:
                    graph_manager.restore_checkpoint()
                last_checkpoint = new_checkpoint


def main():
    screen.set_use_colors(False)

    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--checkpoint_dir',
                        help='(string) Path to a folder containing a checkpoint to restore the model from.',
                        type=str,
                        default='./checkpoint')
    parser.add_argument('--s3_bucket',
                        help='(string) S3 bucket',
                        type=str,
                        default=os.environ.get("SAGEMAKER_SHARED_S3_BUCKET", "gsaur-test"))
    parser.add_argument('--s3_prefix',
                        help='(string) S3 prefix',
                        type=str,
                        default=os.environ.get("SAGEMAKER_SHARED_S3_PREFIX", "sagemaker"))
    parser.add_argument('--num-workers',
                        help="(int) The number of workers started in this pool",
                        type=int,
                        default=1)
    parser.add_argument('-r', '--redis_ip',
                        help="(string) IP or host for the redis server",
                        default='localhost',
                        type=str)
    parser.add_argument('-rp', '--redis_port',
                        help="(int) Port of the redis server",
                        default=6379,
                        type=int)
    parser.add_argument('--aws_region',
                        help='(string) AWS region',
                        type=str,
                        default=os.environ.get("APP_REGION", "us-east-1"))
    parser.add_argument('--reward_file_s3_key',
                        help='(string) Reward File S3 Key',
                        type=str,
                        default=os.environ.get("REWARD_FILE_S3_KEY", None))
    parser.add_argument('--model_metadata_s3_key',
                        help='(string) Model Metadata File S3 Key',
                        type=str,
                        default=os.environ.get("MODEL_METADATA_FILE_S3_KEY", None))

    args = parser.parse_args()

    s3_client = SageS3Client(bucket=args.s3_bucket, s3_prefix=args.s3_prefix, aws_region=args.aws_region)
    print("S3 bucket: %s" % args.s3_bucket)
    print("S3 prefix: %s" % args.s3_prefix)

    # Load the model metadata
    model_metadata_local_path = os.path.join(CUSTOM_FILES_PATH, 'model_metadata.json')
    load_model_metadata(s3_client, args.model_metadata_s3_key, model_metadata_local_path)

    redis_ip = s3_client.get_ip()
    print("Received IP from SageMaker successfully: %s" % redis_ip)

    # Download hyperparameters from SageMaker
    hyperparameters_file_success = False
    hyperparams_s3_key = os.path.normpath(args.s3_prefix + "/ip/hyperparameters.json")
    hyperparameters_file_success = s3_client.download_file(s3_key=hyperparams_s3_key,
                                                           local_path="hyperparameters.json")
    sm_hyperparams_dict = {}
    if hyperparameters_file_success:
        print("Received Sagemaker hyperparameters successfully!")
        with open("hyperparameters.json") as fp:
            sm_hyperparams_dict = json.load(fp)
    else:
        print("SageMaker hyperparameters not found.")

    preset_file_success = False
    environment_file_success = False
    preset_file_success, environment_file_success = download_custom_files_if_present(s3_client, args.s3_prefix)

    if not environment_file_success:
        # Download reward function if environment file is not downloaded
        if not args.reward_file_s3_key:
            raise ValueError("Customer reward S3 key not supplied!")
        download_customer_reward_function(s3_client, args.reward_file_s3_key)
        import markov.environments
        print("Using default environment!")
    else:
        register_custom_environments()
        print("Using custom environment!")

    if preset_file_success:
        preset_location = os.path.join(CUSTOM_FILES_PATH, "preset.py")
        preset_location += ":graph_manager"
        graph_manager = short_dynamic_import(preset_location, ignore_module_case=True)
        print("Using custom preset file!")
    else:
        from markov.sagemaker_graph_manager import get_graph_manager
        graph_manager, _ = get_graph_manager(**sm_hyperparams_dict)

    memory_backend_params = RedisPubSubMemoryBackendParameters(redis_address=redis_ip,
                                                               redis_port=6379,
                                                               run_type='worker',
                                                               channel=args.s3_prefix)

    graph_manager.agent_params.memory.register_var('memory_backend_params', memory_backend_params)

    ds_params_instance = S3BotoDataStoreParameters(bucket_name=args.s3_bucket,
                                                   checkpoint_dir=args.checkpoint_dir, aws_region=args.aws_region,
                                                   s3_folder=args.s3_prefix)

    data_store = S3BotoDataStore(ds_params_instance)
    data_store.graph_manager = graph_manager
    graph_manager.data_store = data_store

    rollout_worker(
        graph_manager=graph_manager,
        checkpoint_dir=args.checkpoint_dir,
        data_store=data_store,
        num_workers=args.num_workers,
    )


if __name__ == '__main__':
    main()
