export MINIO_ACCESS_KEY=minio
export MINIO_SECRET_KEY=miniokey
export AWS_ACCESS_KEY_ID=minio
export AWS_SECRET_ACCESS_KEY=miniokey
export WORLD_NAME=New_York_Track
export ROS_AWS_REGION=us-east-1
export AWS_REGION=us-east-1
export AWS_DEFAULT_REGION=us-east-1
export MODEL_S3_PREFIX=rl-deepracer-sagemaker
export MODEL_S3_BUCKET=bucket
export LOCAL=True

# Check if the "hostname -i" command works
if hostname -i 2>/dev/null ;
then
  export S3_ENDPOINT_URL=http://$(hostname -i):9000  
else  
  # On Macs hostname doesn't support a -i option
  IP_ADDRESS=`ifconfig | grep "inet " | grep -m 1 -Fv 127.0.0.1 | awk '{print $2}'`
  export S3_ENDPOINT_URL=http://${IP_ADDRESS}:9000
fi

export MARKOV_PRESET_FILE=deepracer.py

# Check if the "readlink -f" command works
if [[ -x $(which readlink) ]] && readlink -f ./env_vars.json >/dev/null 2>&1 ;
then
  export LOCAL_ENV_VAR_JSON_PATH=$(readlink -f ./env_vars.json)
elif [[ -x $(which greadlink) ]] && greadlink -f ./env_vars.json >/dev/null 2>&1 ;
then	
  # On Macs the readlink command doesn't support the -f option, so we need to
  # use greadlink instead
  export LOCAL_ENV_VAR_JSON_PATH=$(greadlink -f ./env_vars.json)
else
  echo "Missing readlink and greadlink, cannot set LOCAL_ENV_VAR_JSON_PATH"
  return 1
fi
  
#export LOCAL_EXTRA_DOCKER_COMPOSE_PATH=$(readlink -f ./docker_compose_extra.json)
