#!/usr/bin/env python
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################
import cv2
import logging
import numpy as np
import rospy
import threading
import time

from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from markov.track_geom.track_data import TrackData
from markov.deepracer_exceptions import GenericRolloutException
from markov import utils
from markov.track_geom.utils import euler_to_quaternion, apply_orientation
from markov.utils import Logger
from shapely.geometry import Point
from scipy.integrate import odeint
from sensor_msgs.msg import Image
from threading import Thread
from cv_bridge import CvBridge, CvBridgeError

logger = Logger(__name__, logging.INFO).get_logger()

CAMERA_FPS = 1.0 / 15.0
SCALE_RATIO = 2.5

"""this module is for kinesis camera singelton"""
class KinesisVideoCamera(object):
    _instance_ = None

    @staticmethod
    def get_instance():
        '''Method for geting a reference to the camera object'''
        if KinesisVideoCamera._instance_ is None:
            KinesisVideoCamera()
        return KinesisVideoCamera._instance_

    def __init__(self):
        if KinesisVideoCamera._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple kinesis video camera")
        # init cv bridge
        self.bridge = CvBridge()
        # Double buffer so that we can always publish to KVS, even if physics freezes
        self.sub_camera_frame_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        self.main_camera_frame_buffer = utils.DoubleBuffer(clear_data_on_get=False)

        camera_topic_format = "/{}/zed/rgb/image_rect_color"
        main_camera_topic = camera_topic_format.format("main_camera")
        sub_camera_topic = camera_topic_format.format("sub_camera")
        # This the topic that the camera object publishes too
        rospy.Subscriber(sub_camera_topic, Image, self._sub_camera_cb_)
        rospy.Subscriber(main_camera_topic, Image, self._main_camera_cb_)

        # Create a publisher and new topic for kvs to subscribe to
        self.kvs_pub = rospy.Publisher('/deepracer/kvs_stream', Image, queue_size=1)
        # Run the publisher on its own thread
        Thread(target=self._publish_kvs_frames_).start()
        # there should be only one kinesis video camera
        KinesisVideoCamera._instance_ = self

    def _sub_camera_cb_(self, frame):
        '''Callback for the frames being publish by the top camera topic
            frame - Frames, of type Image, being published by sub camera topic
        '''
        self.sub_camera_frame_buffer.put(frame)

    def _main_camera_cb_(self, frame):
        '''Callback for the frames being publish by the top camera topic
            frame - Frames, of type Image, being published by main camera topic
        '''
        self.main_camera_frame_buffer.put(frame)

    def _overlay_two_images_(self, major_frame, minor_frame):
        # convert ros image message to cv image
        try:
            major_cv_image = self.bridge.imgmsg_to_cv2(major_frame, "bgr8")
            minor_cv_image = self.bridge.imgmsg_to_cv2(minor_frame, "bgr8")
        except CvBridgeError as e:
            logger.info("ROS image message to cv2 error: {}".format(e))

        # overlay minor on the bottom right of major
        rows, cols, _ = minor_cv_image.shape
        minor_cv_image = cv2.resize(minor_cv_image, (int(cols//SCALE_RATIO), int(rows//SCALE_RATIO)))
        major_cv_image[-minor_cv_image.shape[0]:,
                       -minor_cv_image.shape[1]:] = minor_cv_image

        # convert cv image back to ros image message
        try:
            overlay_frame = self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8")
        except CvBridgeError as e:
            logger.info("cv2 to ROS image message error: {}".format(e))

        return overlay_frame


    def _publish_kvs_frames_(self):
        '''This method should be run in its own thread, its used to publish frames
           to the kvs encoder
        '''
        while not rospy.is_shutdown():
            sub_camera_frame = self.sub_camera_frame_buffer.get()
            main_camera_frame = self.main_camera_frame_buffer.get()
            frame = self._overlay_two_images_(main_camera_frame, sub_camera_frame)
            time.sleep(CAMERA_FPS)
            self.kvs_pub.publish(frame)

if __name__ == '__main__':
    rospy.init_node('kinesis_video_camera_node', anonymous=True)
    kinesis_video_camera = KinesisVideoCamera.get_instance()
    rospy.spin()
