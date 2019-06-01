#!/usr/bin/env python
# coding: utf-8

# # Distributed DeepRacer RL training with SageMaker and RoboMaker
# 
# ---
# ## Introduction
# 
# 
# In this notebook, we will train a fully autonomous 1/18th scale race car using reinforcement learning using Amazon SageMaker RL and AWS RoboMaker's 3D driving simulator. [AWS RoboMaker](https://console.aws.amazon.com/robomaker/home#welcome) is a service that makes it easy for developers to develop, test, and deploy robotics applications.  
# 
# This notebook provides a jailbreak experience of [AWS DeepRacer](https://console.aws.amazon.com/deepracer/home#welcome), giving us more control over the training/simulation process and RL algorithm tuning.
# 
# ![Training in Action](./deepracer-hard-track-world.jpg)
# 
# 
# ---
# ## How it works?  
# 
# ![How training works](./training.png)
# 
# The reinforcement learning agent (i.e. our autonomous car) learns to drive by interacting with its environment, e.g., the track, by taking an action in a given state to maximize the expected reward. The agent learns the optimal plan of actions in training by trial-and-error through repeated episodes.  
#   
# The figure above shows an example of distributed RL training across SageMaker and two RoboMaker simulation envrionments that perform the **rollouts** - execute a fixed number of episodes using the current model or policy. The rollouts collect agent experiences (state-transition tuples) and share this data with SageMaker for training. SageMaker updates the model policy which is then used to execute the next sequence of rollouts. This training loop continues until the model converges, i.e. the car learns to drive and stops going off-track. More formally, we can define the problem in terms of the following:  
# 
# 1. **Objective**: Learn to drive autonomously by staying close to the center of the track.
# 2. **Environment**: A 3D driving simulator hosted on AWS RoboMaker.
# 3. **State**: The driving POV image captured by the car's head camera, as shown in the illustration above.
# 4. **Action**: Six discrete steering wheel positions at different angles (configurable)
# 5. **Reward**: Positive reward for staying close to the center line; High penalty for going off-track. This is configurable and can be made more complex (for e.g. steering penalty can be added).

# ## Prequisites

# ### Imports

# To get started, we'll import the Python libraries we need, set up the environment with a few prerequisites for permissions and configurations.

# You can run this notebook from your local machine or from a SageMaker notebook instance. In both of these scenarios, you can run the following to launch a training job on `SageMaker` and a simulation job on `RoboMaker`.

# In[ ]:


import sagemaker
import boto3
import sys
import os
import glob
import re
import subprocess
from IPython.display import Markdown
from time import gmtime, strftime
sys.path.append("common")
from misc import get_execution_role, wait_for_s3_object
from sagemaker.rl import RLEstimator, RLToolkit, RLFramework
from markdown_helper import *


# ### Setup S3 bucket

# Set up the linkage and authentication to the S3 bucket that we want to use for checkpoint and metadata.

# In[ ]:

#Endpoint:  http://10.0.2.15:9000  http://127.0.0.1:9000    
#AccessKey: UNE5P8U6MAL2YTPO9ME4 
#SecretKey: py12zho8+d+y0TRtqC4+pofJRsCRSAu6Tl2bSpXM 

# S3 bucket
boto_session = boto3.session.Session(
    aws_access_key_id="minio", 
    aws_secret_access_key="miniokey",
    region_name="us-east-1")
s3Client = boto_session.resource("s3", use_ssl=False,
endpoint_url=os.environ.get("S3_ENDPOINT_URL", "http://127.0.0.1:9000"))
#s3Client.Bucket("bucket").download_file("rl-deepracer-sagemaker/presets/deepracer.py", "./deepracer.py")
#sys.exit(0)
sage_session = sagemaker.local.LocalSession(boto_session=boto_session, s3_client=s3Client)
s3_bucket = "bucket" #sage_session.default_bucket() 
s3_output_path = 's3://{}/'.format(s3_bucket) # SDK appends the job name and output folder

#sys.exit(0)
# ### Define Variables

# We define variables such as the job prefix for the training jobs and s3_prefix for storing metadata required for synchronization between the training and simulation jobs

# In[ ]:


job_name_prefix = 'rl-deepracer'

# create unique job name
tm = gmtime()
job_name = s3_prefix = job_name_prefix + "-sagemaker"#-" + strftime("%y%m%d-%H%M%S", tm) #Ensure S3 prefix contains SageMaker
s3_prefix_robomaker = job_name_prefix + "-robomaker"#-" + strftime("%y%m%d-%H%M%S", tm) #Ensure that the S3 prefix contains the keyword 'robomaker'


# Duration of job in seconds (5 hours)
job_duration_in_seconds = 24 * 60 * 60

aws_region = sage_session.boto_region_name

if aws_region not in ["us-west-2", "us-east-1", "eu-west-1"]:
    raise Exception("This notebook uses RoboMaker which is available only in US East (N. Virginia), US West (Oregon) and EU (Ireland). Please switch to one of these regions.")
print("Model checkpoints and other metadata will be stored at: {}{}".format(s3_output_path, job_name))


# ### Create an IAM role
# Either get the execution role when running from a SageMaker notebook `role = sagemaker.get_execution_role()` or, when running from local machine, use utils method `role = get_execution_role('role_name')` to create an execution role.

# In[ ]:

'''
try:
    role = sagemaker.get_execution_role()
except:
    role = get_execution_role('sagemaker')

print("Using IAM role arn: {}".format(role))
'''


# > Please note that this notebook cannot be run in `SageMaker local mode` as the simulator is based on AWS RoboMaker service.

# ### Permission setup for invoking AWS RoboMaker from this notebook

# In order to enable this notebook to be able to execute AWS RoboMaker jobs, we need to add one trust relationship to the default execution role of this notebook.
# 

# In[ ]:


#display(Markdown(generate_help_for_robomaker_trust_relationship(role)))


# ### Configure VPC

# Since SageMaker and RoboMaker have to communicate with each other over the network, both of these services need to run in VPC mode. This can be done by supplying subnets and security groups to the job launching scripts.  
# We will use the default VPC configuration for this example.

# In[ ]:

'''
ec2 = boto3.client('ec2')
default_vpc = [vpc['VpcId'] for vpc in ec2.describe_vpcs()['Vpcs'] if vpc["IsDefault"] == True][0]

default_security_groups = [group["GroupId"] for group in ec2.describe_security_groups()['SecurityGroups']                    if group["GroupName"] == "default" and group["VpcId"] == default_vpc]

default_subnets = [subnet["SubnetId"] for subnet in ec2.describe_subnets()["Subnets"]                   if subnet["VpcId"] == default_vpc and subnet['DefaultForAz']==True]

print("Using default VPC:", default_vpc)
print("Using default security group:", default_security_groups)
print("Using default subnets:", default_subnets)


# A SageMaker job running in VPC mode cannot access S3 resourcs. So, we need to create a VPC S3 endpoint to allow S3 access from SageMaker container. To learn more about the VPC mode, please visit [this link.](https://docs.aws.amazon.com/sagemaker/latest/dg/train-vpc.html)

# In[ ]:


try:
    route_tables = [route_table["RouteTableId"] for route_table in ec2.describe_route_tables()['RouteTables']                if route_table['VpcId'] == default_vpc]
except Exception as e:
    if "UnauthorizedOperation" in str(e):
        display(Markdown(generate_help_for_s3_endpoint_permissions(role)))
    else:
        display(Markdown(create_s3_endpoint_manually(aws_region, default_vpc)))
    raise e

print("Trying to attach S3 endpoints to the following route tables:", route_tables)

assert len(route_tables) >= 1, "No route tables were found. Please follow the VPC S3 endpoint creation "                              "guide by clicking the above link."

try:
    ec2.create_vpc_endpoint(DryRun=False,
                           VpcEndpointType="Gateway",
                           VpcId=default_vpc,
                           ServiceName="com.amazonaws.{}.s3".format(aws_region),
                           RouteTableIds=route_tables)
    print("S3 endpoint created successfully!")
except Exception as e:
    if "RouteAlreadyExists" in str(e):
        print("S3 endpoint already exists.")
    elif "UnauthorizedOperation" in str(e):
        display(Markdown(generate_help_for_s3_endpoint_permissions(role)))
        raise e
    else:
        display(Markdown(create_s3_endpoint_manually(aws_region, default_vpc)))
        raise e
'''

# ## Setup the environment
# 

# The environment is defined in a Python file called “deepracer_env.py” and the file can be found at `src/robomaker/environments/`. This file implements the gym interface for our Gazebo based RoboMakersimulator. This is a common environment file used by both SageMaker and RoboMaker. The environment variable - `NODE_TYPE` defines which node the code is running on. So, the expressions that have `rospy` dependencies are executed on RoboMaker only.  
# 
# We can experiment with different reward functions by modifying `reward_function` in this file. Action space and steering angles can be changed by modifying the step method in `DeepRacerDiscreteEnv` class.

# ### Configure the preset for RL algorithm
# The parameters that configure the RL training job are defined in `src/robomaker/presets/deepracer.py`. Using the preset file, you can define agent parameters to select the specific agent algorithm. We suggest using Clipped PPO for this example.  
# You can edit this file to modify algorithm parameters like learning_rate, neural network structure, batch_size, discount factor etc.

# In[ ]:


#get_ipython().system('pygmentize src/robomaker/presets/deepracer.py')


# ### Training Entrypoint
# The training code is written in the file “training_worker.py” which is uploaded in the /src directory. At a high level, it does the following:
# - Uploads SageMaker node's IP address.
# - Starts a Redis server which receives agent experiences sent by rollout worker[s] (RoboMaker simulator).
# - Trains the model everytime after a certain number of episodes are received.
# - Uploads the new model weights on S3. The rollout workers then update their model to execute the next set of episodes.

# In[ ]:


# Uncomment the line below to see the training code
#!pygmentize src/training_worker.py


# ### Train the RL model using the Python SDK Script mode¶
# 

# First, we upload the preset and envrionment file to a particular location on S3, as expected by RoboMaker.

# In[ ]:


s3_location = "s3://%s/%s" % (s3_bucket, s3_prefix)
print("Uploading to " + s3_location)

# Make sure nothing exists at this S3 prefix
#get_ipython().system('aws --endpoint-url http://127.0.0.1:9000 s3 rm --recursive {s3_location}')

# Make any changes to the envrironment and preset files below and upload these files
#print("Executing: " + 'aws --endpoint-url http://127.0.0.1:9000 s3 cp src/robomaker/environments/ {s3_location}/environments/ --recursive --exclude ".ipynb_checkpoints*" --exclude "*.pyc"')
get_ipython().system('aws --endpoint-url http://127.0.0.1:9000 s3 cp src/robomaker/environments/ {s3_location}/environments/ --recursive --exclude ".ipynb_checkpoints*" --exclude "*.pyc"')
get_ipython().system('aws --endpoint-url http://127.0.0.1:9000 s3 cp src/robomaker/presets/ {s3_location}/presets/ --recursive --exclude ".ipynb_checkpoints*" --exclude "*.pyc"')


# Next, we define the following algorithm metrics that we want to capture from cloudwatch logs to monitor the training progress. These are algorithm specific parameters and might change for different algorithm. We use [Clipped PPO](https://coach.nervanasys.com/algorithms/policy_optimization/cppo/index.html) for this example.

# In[ ]:


metric_definitions = [
    # Training> Name=main_level/agent, Worker=0, Episode=19, Total reward=-102.88, Steps=19019, Training iteration=1
    {'Name': 'reward-training',
     'Regex': '^Training>.*Total reward=(.*?),'},
    
    # Policy training> Surrogate loss=-0.32664725184440613, KL divergence=7.255815035023261e-06, Entropy=2.83156156539917, training epoch=0, learning_rate=0.00025
    {'Name': 'ppo-surrogate-loss',
     'Regex': '^Policy training>.*Surrogate loss=(.*?),'},
     {'Name': 'ppo-entropy',
     'Regex': '^Policy training>.*Entropy=(.*?),'},
   
    # Testing> Name=main_level/agent, Worker=0, Episode=19, Total reward=1359.12, Steps=20015, Training iteration=2
    {'Name': 'reward-testing',
     'Regex': '^Testing>.*Total reward=(.*?),'},
]


# We use the RLEstimator for training RL jobs.
# 
# 1. Specify the source directory which has the environment file, preset and training code.
# 2. Specify the entry point as the training code
# 3. Specify the choice of RL toolkit and framework. This automatically resolves to the ECR path for the RL Container.
# 4. Define the training parameters such as the instance count, instance type, job name, s3_bucket and s3_prefix for storing model checkpoints and metadata. **Only 1 training instance is supported for now.**
# 4. Set the RLCOACH_PRESET as "deepracer" for this example.
# 5. Define the metrics definitions that you are interested in capturing in your logs. These can also be visualized in CloudWatch and SageMaker Notebooks.

# In[ ]:


RLCOACH_PRESET = "deepracer"

instance_type = "local"


estimator = RLEstimator(entry_point="training_worker.py",
                        source_dir='src',
                        dependencies=["common/sagemaker_rl"],
                        toolkit=RLToolkit.COACH,
                        toolkit_version='0.11',
                        framework=RLFramework.TENSORFLOW,
                        sagemaker_session=sage_session,
                        #bypass sagemaker SDK validation of the role
                        role="aaa/",
                        train_instance_type=instance_type,
                        train_instance_count=1,
                        output_path=s3_output_path,
                        base_job_name=job_name_prefix,
                        train_max_run=job_duration_in_seconds, # Maximum runtime in seconds
                        hyperparameters={"s3_bucket": s3_bucket,
                                         "s3_prefix": s3_prefix,
                                         "aws_region": aws_region,
                                         "model_metadata_s3_key": "s3://bucket/custom_files/model_metadata.json",
                                         "RLCOACH_PRESET": RLCOACH_PRESET,
                                         "loss_type": "mean squared error"
                                         #"pretrained_s3_bucket": "bucket",
                                         #"pretrained_s3_prefix": "rl-deepracer-pretrained"
                                      },
                        metric_definitions = metric_definitions,
						s3_client=s3Client
                        #subnets=default_subnets, # Required for VPC mode
                        #security_group_ids=default_security_groups, # Required for VPC mode
                    )

estimator.fit(job_name=job_name, wait=False)


# ### Start the Robomaker job

# In[ ]:

'''
from botocore.exceptions import UnknownServiceError

robomaker = boto3.client("robomaker")


# ### Create Simulation Application

# We first create a RoboMaker simulation application using the `DeepRacer public bundle`. Please refer to [RoboMaker Sample Application Github Repository](https://github.com/aws-robotics/aws-robomaker-sample-application-deepracer) if you want to learn more about this bundle or modify it.

# In[ ]:


bundle_s3_key = 'deepracer/simulation_ws.tar.gz'
bundle_source = {'s3Bucket': s3_bucket,
                 's3Key': bundle_s3_key,
                 'architecture': "X86_64"}
simulation_software_suite={'name': 'Gazebo',
                           'version': '7'}
robot_software_suite={'name': 'ROS',
                      'version': 'Kinetic'}
rendering_engine={'name': 'OGRE', 'version': '1.x'}


# Download the public DeepRacer bundle provided by RoboMaker and upload it in our S3 bucket to create a RoboMaker Simulation Application

# In[ ]:


simulation_application_bundle_location = "https://s3.amazonaws.com/deepracer-managed-resources/deepracer-github-simapp.tar.gz"

get_ipython().system('wget {simulation_application_bundle_location}')
get_ipython().system('aws --endpoint-url http://127.0.0.1:9000 s3 cp deepracer-github-simapp.tar.gz s3://{s3_bucket}/{bundle_s3_key}')
get_ipython().system('rm deepracer-github-simapp.tar.gz')


# In[ ]:

app_name = "deepracer-sample-application"# + strftime("%y%m%d-%H%M%S", gmtime())

try:
    response = robomaker.create_simulation_application(name=app_name,
                                                   sources=[bundle_source],
                                                   simulationSoftwareSuite=simulation_software_suite,
                                                   robotSoftwareSuite=robot_software_suite,
                                                   renderingEngine=rendering_engine
                                                  )
    simulation_app_arn = response["arn"]
    print("Created a new simulation app with ARN:", simulation_app_arn)
except Exception as e:
    if "AccessDeniedException" in str(e):
        display(Markdown(generate_help_for_robomaker_all_permissions(role)))
        raise e
    else:
        raise e

# ### Launch the Simulation job on RoboMaker
# 
# We create [AWS RoboMaker](https://console.aws.amazon.com/robomaker/home#welcome) Simulation Jobs that simulates the environment and shares this data with SageMaker for training. 

# In[ ]:


# Use more rollout workers for faster convergence
num_simulation_workers = 1

envriron_vars = {
                 "MODEL_S3_BUCKET": s3_bucket,
                 "MODEL_S3_PREFIX": s3_prefix,
                 "ROS_AWS_REGION": aws_region,
                 "WORLD_NAME": "hard_track",  # Can be one of "easy_track", "medium_track", "hard_track"
                 "MARKOV_PRESET_FILE": "%s.py" % RLCOACH_PRESET,
                 "NUMBER_OF_ROLLOUT_WORKERS": str(num_simulation_workers)}

simulation_application = {"application": simulation_app_arn,
                          "launchConfig": {"packageName": "deepracer_simulation",
                                           "launchFile": "distributed_training.launch",
                                           "environmentVariables": envriron_vars}
                         }
                            
vpcConfig = {"subnets": default_subnets,
             "securityGroups": default_security_groups,
             "assignPublicIp": True}

responses = []
for job_no in range(num_simulation_workers):
    response =  robomaker.create_simulation_job(iamRole=role,
                                            clientRequestToken=strftime("%Y-%m-%d-%H-%M-%S", gmtime()),
                                            maxJobDurationInSeconds=job_duration_in_seconds,
                                            failureBehavior="Continue",
                                            simulationApplications=[simulation_application],
                                            vpcConfig=vpcConfig,
                                            outputLocation={"s3Bucket":s3_bucket, "s3Prefix":s3_prefix_robomaker}
                                            )
    responses.append(response)

print("Created the following jobs:")
job_arns = [response["arn"] for response in responses]
for job_arn in job_arns:
    print("Job ARN", job_arn) 


# ### Visualizing the simulations in RoboMaker

# You can visit the RoboMaker console to visualize the simulations or run the following cell to generate the hyperlinks.

# In[ ]:


display(Markdown(generate_robomaker_links(job_arns, aws_region)))


# ### Plot metrics for training job

# In[ ]:


tmp_dir = "/tmp/{}".format(job_name)
os.system("mkdir {}".format(tmp_dir))
print("Create local folder {}".format(tmp_dir))
intermediate_folder_key = "{}/output/intermediate".format(job_name)


# In[ ]:


get_ipython().run_line_magic('matplotlib', 'inline')
import pandas as pd

csv_file_name = "worker_0.simple_rl_graph.main_level.main_level.agent_0.csv"
key = intermediate_folder_key + "/" + csv_file_name
wait_for_s3_object(s3_bucket, key, tmp_dir)

csv_file = "{}/{}".format(tmp_dir, csv_file_name)
df = pd.read_csv(csv_file)
df = df.dropna(subset=['Training Reward'])
x_axis = 'Episode #'
y_axis = 'Training Reward'

plt = df.plot(x=x_axis,y=y_axis, figsize=(12,5), legend=True, style='b-')
plt.set_ylabel(y_axis);
plt.set_xlabel(x_axis);


# ### Clean Up

# Execute the cells below if you want to kill RoboMaker and SageMaker job.

# In[ ]:

'''
'''
for job_arn in job_arns:
    robomaker.cancel_simulation_job(job=job_arn)


# In[ ]:


sage_session.sagemaker_client.stop_training_job(TrainingJobName=estimator._current_job_name)


# ### Evaluation

# In[ ]:


envriron_vars = {"MODEL_S3_BUCKET": s3_bucket,
                 "MODEL_S3_PREFIX": s3_prefix,
                 "ROS_AWS_REGION": aws_region,
                 "NUMBER_OF_TRIALS": str(20),
                 "MARKOV_PRESET_FILE": "%s.py" % RLCOACH_PRESET,
                 "WORLD_NAME": "hard_track",
                 }

simulation_application = {"application":simulation_app_arn,
                          "launchConfig": {"packageName": "deepracer_simulation",
                                           "launchFile": "evaluation.launch",
                                           "environmentVariables": envriron_vars}
                         }
                            
vpcConfig = {"subnets": default_subnets,
             "securityGroups": default_security_groups,
             "assignPublicIp": True}

response =  robomaker.create_simulation_job(iamRole=role,
                                        clientRequestToken=strftime("%Y-%m-%d-%H-%M-%S", gmtime()),
                                        maxJobDurationInSeconds=job_duration_in_seconds,
                                        failureBehavior="Continue",
                                        simulationApplications=[simulation_application],
                                        vpcConfig=vpcConfig,
                                        outputLocation={"s3Bucket":s3_bucket, "s3Prefix":s3_prefix_robomaker}
                                        )
print("Created the following job:")
print("Job ARN", response["arn"])


# ### Clean Up Simulation Application Resource

# In[ ]:

robomaker.delete_simulation_application(application=simulation_app_arn)
'''
