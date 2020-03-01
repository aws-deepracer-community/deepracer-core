cmake_minimum_required(VERSION 2.8.3)
project(deepracer_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_service_files(
  FILES
  GetWaypointSrv.srv
  ResetCarSrv.srv
)

add_message_files(DIRECTORY msg
                  FILES
                  AgentRewardData.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package()
#############
## Install ##
#############

install(PROGRAMS
    scripts/servo_commands.py
    scripts/run_local_rl_agent.sh
    scripts/run_rollout_rl_agent.sh
    scripts/run_evaluation_rl_agent.sh
    scripts/car_node.py
    scripts/rotation.py
    scripts/download_params_and_roslaunch_agent.py
    scripts/kinesis_video_camera_node.py
    scripts/visualization_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
install(DIRECTORY routes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/routes
)
install(DIRECTORY srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
)
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
