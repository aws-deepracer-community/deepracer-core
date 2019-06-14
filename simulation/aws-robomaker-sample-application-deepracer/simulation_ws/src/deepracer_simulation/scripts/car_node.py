#!/usr/bin/env python
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################

'''This module will launch a ROS node that will have services for retrieving the way
   points and resetting the car. It should serve as an interface into gazebo related
   operations required at start up
'''
import os
import sys
import math
import time

import rospy
import numpy as np

from deepracer_simulation_environment.srv import GetWaypointSrvResponse, GetWaypointSrv
from deepracer_simulation_environment.srv import ResetCarSrv, ResetCarSrvResponse

from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from shapely.geometry.polygon import LinearRing, LineString
from rotation import Rotation

# Service name for the service that allows clients to retrieve the way points
WAYPOINT_SRV_NAME = '/deepracer_simulation_environment/get_waypoints'
# Service name for the service that allows clients to reset the car
RESET_CAR_SRV_NAME = '/deepracer_simulation_environment/reset_car'
# The environment variable that specifies the bundle prefix
BUNDLE_KEY = "BUNDLE_CURRENT_PREFIX"
# Amount of time (in seconds) to wait, in order to prevent model state from
# spamming logs while the model is loading
WAIT_TO_PREVENT_SPAM = 2

class DeepRacer(object):
    def __init__(self):
        ''' Constructor for the Deep Racer object, will load track and waypoints
        '''
        # Wait for required services to be available
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        # We have no guarantees as to when gazebo will load the model, therefore we need
        # to wait until the model is loaded prior to resetting it for the first time
        get_model_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        wait_for_model = True
        while wait_for_model:
            # If the model is not loaded the get model service will log an error
            # Therefore, we add an timer to prevent the log from getting spammed with
            # errors
            time.sleep(WAIT_TO_PREVENT_SPAM)
            model = get_model_client('racecar', '')
            wait_for_model = not model.success

        try:
            bundle_prefix = os.environ.get(BUNDLE_KEY, None)
            route_file_name = os.path.join(bundle_prefix, 'opt', 'install',
                                           'deepracer_simulation_environment', 'share',
                                           'deepracer_simulation_environment', 'routes',
                                           '{}.npy'.format(rospy.get_param('WORLD_NAME')))

            self.waypoints = np.load(route_file_name)
        except Exception as ex:
            print('[ERROR] Unable to import the waypoints {}'.format(ex))
            sys.exit(1)
        # Find the centerline
        is_loop = np.all(self.waypoints[0, :] == self.waypoints[-1, :])
        self.center_line = LinearRing(self.waypoints[:, 0:2]) if is_loop \
                           else LineString(self.waypoints[:, 0:2])
        # Create the service that allows clients to retrieve the waypoints
        self.waypoints_service = rospy.Service(WAYPOINT_SRV_NAME, GetWaypointSrv,
                                               self.handle_get_waypoints)
        # Create service that allows clients to reset the car
        self.resetcar_service = rospy.Service(RESET_CAR_SRV_NAME, ResetCarSrv,
                                              self.handle_reset_car)
        # Gazebo service that allows us to position the car
        self.model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # Place the car at the starting point facing the forward direction
        self.reset_car(0, 1)

    def reset_car(self, ndist, next_index):
        ''' Reset's the car on the track
            ndist - normalized track distance
            next_index - index of the next way point
        '''
        # Compute the starting position and heading
        start_point = self.center_line.interpolate(ndist, normalized=True)
        start_yaw = math.atan2(self.center_line.coords[next_index][1] - start_point.y,
                               self.center_line.coords[next_index][0] - start_point.x)
        start_quaternion = Rotation.from_euler('zyx', [start_yaw, 0, 0]).as_quat()

        # Construct the model state and send to Gazebo
        model_state = ModelState()
        model_state.model_name = 'racecar'
        model_state.pose.position.x = start_point.x
        model_state.pose.position.y = start_point.y
        model_state.pose.position.z = 0
        model_state.pose.orientation.x = start_quaternion[0]
        model_state.pose.orientation.y = start_quaternion[1]
        model_state.pose.orientation.z = start_quaternion[2]
        model_state.pose.orientation.w = start_quaternion[3]
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0
        self.model_state_client(model_state)

    def handle_get_waypoints(self, req):
        '''Request handler for get way point, unconditionally returns the waypoint list
           req - Request from the client which is empty for this service
        '''
        # Unfortunately, there is no clean way to send a numpy array through the ROS system.
        # Therefore, we need to unroll the array and let the client recreate and reshape the
        # numpy object
        waypoint_shape = self.waypoints.shape
        return GetWaypointSrvResponse(waypoint_shape[0], waypoint_shape[1],
                                      self.waypoints.ravel().tolist())

    def handle_reset_car(self, req):
        '''Request handler for resetting the car
           req - Request from the client, should contain ndist and next_index
        '''
        self.reset_car(req.ndist, req.next_index)
        return ResetCarSrvResponse(1)

if __name__ == '__main__':
    rospy.init_node('car_reset_node', anonymous=True)
    DEEPRACER = DeepRacer()
    rospy.spin()
