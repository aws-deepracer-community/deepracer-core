import argparse
import os
import sys
import json

import markov.environments
from markov.s3_client import SageS3Client
from markov.s3_boto_data_store import S3BotoDataStore, S3BotoDataStoreParameters
from markov.utils import load_model_metadata
from rl_coach.base_parameters import TaskParameters
from rl_coach.core_types import EnvironmentEpisodes, RunPhase

CUSTOM_FILES_PATH = "./custom_files"
if not os.path.exists(CUSTOM_FILES_PATH):
    os.makedirs(CUSTOM_FILES_PATH)


def evaluation_worker(graph_manager, number_of_trials, local_model_directory):
    # initialize graph
    task_parameters = TaskParameters()
    task_parameters.__dict__['checkpoint_restore_dir'] = local_model_directory
    graph_manager.create_graph(task_parameters)

    with graph_manager.phase_context(RunPhase.TEST):
        # reset all the levels before starting to evaluate
        graph_manager.reset_internal_state(force_environment_reset=True)
        graph_manager.act(EnvironmentEpisodes(number_of_trials))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--preset',
                        help="(string) Name of a preset to run (class name from the 'presets' directory.)",
                        type=str,
                        required=False)
    parser.add_argument('--s3_bucket',
                        help='(string) S3 bucket',
                        type=str,
                        default=os.environ.get("MODEL_S3_BUCKET", "gsaur-test"))
    parser.add_argument('--s3_prefix',
                        help='(string) S3 prefix',
                        type=str,
                        default=os.environ.get("MODEL_S3_PREFIX", "sagemaker"))
    parser.add_argument('--aws_region',
                        help='(string) AWS region',
                        type=str,
                        default=os.environ.get("APP_REGION", "us-east-1"))
    parser.add_argument('--number_of_trials',
                        help='(integer) Number of trials',
                        type=int,
                        default=os.environ.get("NUMBER_OF_TRIALS", 10))
    parser.add_argument('-c', '--local_model_directory',
                        help='(string) Path to a folder containing a checkpoint to restore the model from.',
                        type=str,
                        default='./checkpoint')

    args = parser.parse_args()

    s3_client = SageS3Client(bucket=args.s3_bucket, s3_prefix=args.s3_prefix, aws_region=args.aws_region)
    print("S3 bucket: %s" % args.s3_bucket)
    print("S3 prefix: %s" % args.s3_prefix)

    # Load the model metadata
    model_metadata_local_path = os.path.join(CUSTOM_FILES_PATH, 'model_metadata.json')
    load_model_metadata(s3_client, os.path.normpath("%s/model/model_metadata.json" % args.s3_prefix), model_metadata_local_path)

    # Download the model
    s3_client.download_model(args.local_model_directory)

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

    from markov.sagemaker_graph_manager import get_graph_manager
    graph_manager, _ = get_graph_manager(**sm_hyperparams_dict)

    ds_params_instance = S3BotoDataStoreParameters(bucket_name=args.s3_bucket,
                                                   checkpoint_dir=args.local_model_directory,
                                                   aws_region=args.aws_region,
                                                   s3_folder=args.s3_prefix)

    data_store = S3BotoDataStore(ds_params_instance)
    graph_manager.data_store = data_store

    evaluation_worker(
        graph_manager=graph_manager,
        number_of_trials=args.number_of_trials,
        local_model_directory=args.local_model_directory
    )


if __name__ == '__main__':
    main()
