#!/usr/bin/env python

# *WARNING*
# THIS SCRIPT MODIFIES FILES ON YOUR DRIVE AND IN YOUR
# AWS S3 BUCKET (DELETES AND UPLOADS).
# WHILE THE AUTHORS HAVE TAKEN CARE TO NOT MAKE IT HARMFUL,
# THEY TAKE NO RESPONSIBILITY FOR ANY DAMAGES IT MAY CAUSE,
# ESPECIALLY (BUT NOT ONLY) IF MISCONFIGURED.
# USE AT YOUR OWN RISK.

import shutil
import os
import glob
import boto3
from datetime import datetime
import logging

default_config_file = 'dr_util_config.json'
config = {}


def init_config(config_file=default_config_file):
    if os.path.exists(config_file):
        logging.warning("%s exists, doing nothing", config_file)
        return

    import json
    with open(config_file, 'w') as f:
        empty_config = {
            'minio_bucket_base': 'path_to_the_minio_folder_ending_with/bucket',
            'aws_dr_bucket': 'name_of_your_deepracer_bucket_in_S3',
            'aws_model_folder': 'name_of_the_folder_in_the_S3_bucket_where_files_are_for_the_model_you_are_submitting',
            'training_folder': 'rl-deepracer-sagemaker',
            'pretrained_folder': 'rl-deepracer-pretrained',
            'custom_files_folder': 'custom_files',
        }
        json.dump(empty_config, f)
        logging.warning("%s created, edit it with your values before continuing", config_file)


def load_config(config_file=default_config_file):
    if not os.path.exists(config_file):
        logging.warning("%s doesn't exist, doing nothing. Initialize your "
                        "config first running command init, see --help",
                        config_file)
        return False
    import json

    global config
    with open(config_file) as f:
        config = json.load(f)

        config['training_path'] = config['minio_bucket_base'] + '/' + config['training_folder']
        config['pretrained_path'] = config['minio_bucket_base'] + '/' + config['pretrained_folder']
        config['custom_files'] = config['minio_bucket_base'] + '/' + config['custom_files_folder']

    return True


def prepare_new_base_folder(base_path=None):
    if not base_path:
        base_path = config['pretrained_path']

    if os.path.exists(base_path):
        folder_aside = base_path + '_' + date_string()
        logging.warning("%s exists, moving aside to %s", base_path, folder_aside)
        os.rename(base_path, folder_aside)
    os.makedirs(base_path + '/model')
    os.makedirs(base_path + '/ip')
    logging.warning("Created folders model and ip in %s", base_path)


def date_string():
    return datetime.now().strftime('%Y%m%d_%H%M%S')


def archive_current_training(suffix=None):
    if not suffix:
        suffix = date_string()
    destination = config['pretrained_path'] + '-' + suffix
    logging.warning("Moving %s to %s", config['training_path'], destination)
    shutil.copy2(config['custom_files'] + '/reward.py', config['training_path'] + '/ip/reward.py')
    os.rename(config['training_path'], destination)


def snapshot_a_training(base_model=None, suffix=None, label=None):
    if suffix is None:
        suffix = date_string()
    destination = '/rl-deepracer-pretrained'
    if suffix:
        if label is None:
            label = suffix
        destination += '-' + suffix
    destination_folder = config['minio_bucket_base'] + destination

    prepare_new_base_folder(base_path=destination_folder)

    if not base_model:
        base_model = config['training_path']
    else:
        with open(destination_folder + '/ip/based_on.txt', 'w') as f:
            f.write(base_model)

    metas = glob.glob(base_model + '/model/*_Step-*.ckpt.meta')
    metas.sort(key=by_step_index, reverse=True)

    name_base = metas[0].split('/')[-1].replace('.meta', '')

    if not label:
        label = name_base
    elif not label.endswith('.ckpt'):
        label = '0_Step-' + label + '.ckpt'

    source_base = base_model + '/model/' + name_base
    destination_base = destination_folder + '/model/' + label

    logging.warning("Creating a snapshot of %s with label %s", source_base, label)

    for extension in ['.data-00000-of-00001', '.index', '.meta']:
        shutil.copy2(source_base + extension, destination_base + extension)

    shutil.copy2(base_model + '/ip/hyperparameters.json', destination_folder + '/ip/hyperparameters.json')
    shutil.copy2(config['custom_files'] + '/reward.py', destination_folder + '/ip/reward.py')
    if os.path.exists(config['custom_files'] + '/based_on.txt'):
        shutil.copy2(config['custom_files'] + '/based_on.txt', destination_folder + '/ip/based_on.txt')

    shutil.copy2(base_model + '/model/model_metadata.json',
                 destination_folder + '/model/model_metadata.json')

    with open(destination_folder + '/model/checkpoint', 'w') as f:
        f.write('model_checkpoint_path: "{ckpt}"\nall_model_checkpoint_paths: "{ckpt}"\n'.format(ckpt=label))


def by_step_index(filename):
    return int(filename.split('/')[-1].split('_')[0])


def upload_model(model_base, bucket_name=None, folder=None):
    if not bucket_name:
        bucket_name = config['aws_dr_bucket']
    if not folder:
        folder = config['aws_model_folder']
    model_files = glob.glob(model_base + '/model/*')
    if len(model_files) == 0:
        logging.warning("No model found in %s, doing nothing", model_base)
        return
    if len(model_files) > 5:
        logging.warning("You are trying to upload the whole training output. "
                        "While you could, this would be a lot of data, so you cannot. "
                        "Make a snapshot first")
        return
    s3 = boto3.resource('s3')
    bucket = s3.Bucket(bucket_name)
    logging.warning('Deleting old model files:')
    for file in bucket.objects.filter(Prefix=folder + '/model'):
        logging.warning(file)
        file.delete()

    logging.warning('Uploading new model files')
    for file in model_files:
        logging.warning(file)
        bucket.upload_file(file, folder + '/model/' + file.split('/')[-1])


actions = ['init', 'archive', 'snapshot', 'upload']


def parse_options():
    global parser, options, args
    from optparse import OptionParser, OptionGroup
    parser = OptionParser(usage='usage: %prog [options]')
    parser.add_option('-a', '--action', action='store', type='string', dest='action',
                      help='Action to perform, one of: ' + ', '.join(actions))
    archive_group = OptionGroup(parser, 'Archive task Options',
                                'These are used when [-a archive] is selected.\n'
                                'This action moves rl-deepracer-sagemaker to rl-seepracer-pretrained-<suffix>')
    archive_group.add_option('--asuffix', action='store',
                             help='What suffix to give. If none is given, a time suffix will be used')
    parser.add_option_group(archive_group)
    snapshot_group = OptionGroup(parser, 'Snapshot task Options',
                                 'These are used when [-a snapshot] is selected.\n'
                                 'This action takes the latest step from rl-deepracer-sagemaker and creates a snapshot '
                                 'of a model in rl-seepracer-pretrained-<suffix>. The file is labeled with <label> '
                                 'which is handy as the label will be written in the evaluation logs')
    snapshot_group.add_option('--smodel', action='store',
                              help='What model to snapshot. If none is given, a currently trained model is used')
    snapshot_group.add_option('--ssuffix', action='store',
                              help='What suffix to give. If none is given, a time suffix will be used')
    snapshot_group.add_option('--slabel', action='store',
                              help='What label to give to the Step file. If none is given, the actual '
                                   'Step file name will be used')
    parser.add_option_group(snapshot_group)
    upload_group = OptionGroup(parser, 'Upload task Options',
                               'These are used when [-a upload] is selected.\n'
                               'This action uploads a model to AWS S3 bucket\n'
                               'Make sure you configured the dr_util script')
    upload_group.add_option('--umodel', action='store',
                            help='Path to the model that you want to upload\n'
                                 'This will work with a whole model, but I strongly '
                                 'advice to use snapshots - less to upload')
    parser.add_option_group(upload_group)
    options, args = parser.parse_args()

    if not options.action or options.action not in actions:
        parser.error('Action not set properly, I don\'t know what to do')

    if options.action == 'upload' and not options.umodel:
        parser.error('Provide path to model to upload')

    return options, args


if __name__ == '__main__':
    parse_options()

    if not load_config():
        exit(1)

    if options.action == 'init':
        init_config()
    elif options.action == 'archive':
        archive_current_training(options.asuffix)
    elif options.action == 'snapshot':
        snapshot_a_training(options.smodel, options.ssuffix, options.slabel)
    elif options.action == 'upload':
        upload_model(options.umodel)
