#!/usr/bin/env python

# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os

import rospy
import message_filters
import cv2
import numpy as np

from std_msgs.msg import String, Int16

import argparse
import time


class OpenPoseSubscriber():
    def __init__(self):
        rospy.init_node('openpose_people_subscriber')

        # self.warned_about_one_people

        self.depth_info_subscriber = rospy.Subscriber(
            'openpose/close_people', Int16, self.talk, queue_size=10
            )

        rospy.loginfo('OpenPose People Subscriber is done')


    def talk(self, people_code: Int16):
        # rospy.loginfo(f'Received info')

        if people_code.data == 0:
            pass
        if people_code.data == 1:
            # tag Ilya
            rospy.loginfo('Someone came close to the robot')
            rospy.sleep(30.)
        if people_code.data == 2:
            # tag Ilya
            rospy.loginfo('Robot is surrounded by people')
            rospy.sleep(30.)


if __name__ == '__main__':
    try:
        # parser = argparse.ArgumentParser()

        openpose_people_subscriber = OpenPoseSubscriber()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass