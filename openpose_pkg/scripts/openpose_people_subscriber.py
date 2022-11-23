#!/usr/bin/env python

import rospy

from std_msgs.msg import String

# import argparse

import requests
import time


class OpenPoseSubscriber():
    def __init__(self):
        rospy.init_node('openpose_people_subscriber')

        self.delay = 30
        self.time = time.time() - self.delay

        # self.warned_about_one_people

        self.depth_info_subscriber = rospy.Subscriber(
            'openpose/close_people', String, self.talk, queue_size=10
            )

        rospy.loginfo('OpenPose People Subscriber is done')


    def talk(self, people_code: String):
        # rospy.loginfo(f'Received info')

        if time.time() - self.time >= self.delay:
            if people_code.data == 'no_close':
                pass

            if people_code.data == 'z1':
                requests.get('http://localhost:8000/tts/1')
                rospy.loginfo('Someone came close to the robot')
                self.time = time.time()
                rospy.loginfo('Someone have 30 seconds')

            if people_code.data == 'b1':
                requests.get('http://localhost:8000/tts/1')
                rospy.loginfo('Someone came to the robot from back')
                self.time = time.time()
                rospy.loginfo('Someone have 30 seconds')

            if people_code.data == 'z2' or people_code.data == 'b2':
                requests.get('http://localhost:8000/tts/2')
                rospy.loginfo('Robot is surrounded by people')
                self.time = time.time()
                rospy.loginfo('People have 30 seconds')


if __name__ == '__main__':
    try:
        openpose_people_subscriber = OpenPoseSubscriber()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass