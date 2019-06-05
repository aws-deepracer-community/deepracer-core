# DeepRacer with Amazon SageMaker RL and AWS RoboMaker

This folder contains examples of how to use RL to train an autonomous race car.


## Contents

* `rl_deepracer_clippedppo_coach_tensorflow_robomaker.py`: script for training autonomous race car.


* `src/`
  * `training_worker.py`: Main entrypoint for starting distributed training job
  * `markov/`: Helper files for S3 upload/download
