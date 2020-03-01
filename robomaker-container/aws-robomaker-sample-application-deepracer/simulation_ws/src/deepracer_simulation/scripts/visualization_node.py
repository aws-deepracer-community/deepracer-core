#!/usr/bin/env python
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################

'''This module will launch a ROS node that will have publish visualization topics.
'''

import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import rospy
from sensor_msgs.msg import Image as sensor_image
from deepracer_simulation.msg import AgentRewardData 
from cv_bridge import CvBridge, CvBridgeError
import cv2

class RewardDistributionBarGraph(object):
    def __init__(self,title="Reward Distribution Bar Graph"):
        self.agent_name = 'racecar'
        rospy.Subscriber("/reward_data/"+self.agent_name, AgentRewardData, self.reward_data_subscriber)
        self.reward_distribution_graph_publisher = rospy.Publisher('/visualization/reward_bar_graph/'+self.agent_name, sensor_image, queue_size=1)
        self.bridge = CvBridge()
        
        self.num_of_past_frames = int(rospy.get_param("VIS_NUMBER_OF_PAST_FRAMES", 10))
        # TODO: Change from list to queue
        self.reward_list = [0 for i in range(self.num_of_past_frames)]
        self.action_index_list = [0 for i in range(self.num_of_past_frames)]
        self.image_list = [np.zeros((120, 160)) for i in range(self.num_of_past_frames)]
        self.fig = plt.figure(figsize=(int(rospy.get_param("VIS_FIGURE_WIDTH", 12)),int(rospy.get_param("VIS_FIGURE_HEIGHT", 6))))

        # Define the plot specifics
        gs = gridspec.GridSpec(6, self.num_of_past_frames)
        self.image_axes = []
        for i in range(self.num_of_past_frames):
            ax = self.fig.add_subplot(gs[0, i])
            ax.set_xticks([]) 
            ax.set_yticks([]) 
            im_ax = ax.imshow(self.image_list[i])
            self.image_axes.append(im_ax)

        self.reward_axes = self.fig.add_subplot(gs[1, :])
        self.reward_axes.set_xticks([])
        self.reward_axes.set_ylabel('Reward')
        self.reward_line, = self.reward_axes.plot(self.reward_list)

        frame_count = np.arange(-self.num_of_past_frames+1, 1)

        self.action_index_axes = self.fig.add_subplot(gs[2:, :])
        self.action_index_axes.set_xlabel('Past frame count')
        self.action_index_axes.set_ylabel('Action')
        self.action_index_axes.set_xticks(frame_count)
        self.action_bar_rects = self.action_index_axes.bar(frame_count, self.action_index_list, alpha=0.5)

    def reward_data_subscriber(self, data):
        action_index = data.action
        reward_value = data.reward
        action_space_len = data.action_space_len
        image = data.image
        if image is None:
            image = np.zeros((120, 160))
        else:
            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        if not reward_value:
            reward_value = 0.0
        if not action_index:
            action_index = 0

        self.image_list.pop(0)
        self.image_list += [image]

        self.reward_list.pop(0)
        self.reward_list += [reward_value]

        self.action_index_list.pop(0)
        self.action_index_list += [action_index+1]

        for i in range(self.num_of_past_frames):
            self.image_axes[i].set_data(self.image_list[i])
        min_reward =  min(self.reward_list)
        max_reward = max(self.reward_list)
        self.reward_axes.set_ylim((min_reward - 0.1*min_reward, max_reward + 0.1*max_reward))
        self.reward_line.set_ydata(self.reward_list)

        action_indices = np.arange(0, action_space_len+1)
        action_space_names = ['--']
        for i in range(action_space_len):
            action_space_names.append(data.steering_angle_list[i] + '/' + data.speed_list[i])

        self.action_index_axes.set_ylim((0, action_space_len+2))
        self.action_index_axes.set_yticks(action_indices)
        self.action_index_axes.set_yticklabels(action_space_names)

        for rect, h in zip(self.action_bar_rects, self.action_index_list):
            rect.set_height(h)

        self.fig.canvas.draw()

        data = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        data = data.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))

        self.reward_distribution_graph_publisher.publish(self.bridge.cv2_to_imgmsg(data, "bgr8"))


if __name__ == '__main__':
    rospy.init_node('visualization_node', anonymous=True)
    visualization = RewardDistributionBarGraph()
    rospy.spin()
    