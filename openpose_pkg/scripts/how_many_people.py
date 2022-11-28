#!/usr/bin/env python

import argparse

import rospy
from std_msgs.msg import String

def tell_n_people(info):
    n_people = info.split()[1]
    rospy.loginfo(f'Openpose detected {n_people} people')

if __name__ == '__main__':
    timeout = 10
    try:
        rospy.init_node('openpose_n_people_talker')
        data = rospy.wait_for_message(
            'openpose/detection', String, timeout=timeout
            ).data
        tell_n_people(data)

    except rospy.exceptions.ROSException as exception:
        if exception.args[0].split()[0] == 'timeout':
            rospy.loginfo(f'Openpose did not work for {timeout} seconds, stop waiting')
        else:
            print(exception)

    except rospy.ROSInterruptException as exception:
        print(exception)