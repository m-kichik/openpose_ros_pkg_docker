#!/usr/bin/env python

from __future__ import print_function

import rospy
from openpose_pkg.srv import MyDepth

def return_42(my_message):
    rospy.loginfo(f'Caught message: {my_message.str}')
    rospy.loginfo('Returning depth matrix')
    return 42

def my_depth_server():
    rospy.init_node('depth_server')
    s = rospy.Service('depth_matrix', MyDepth, return_42)
    print("Ready to send depth matrix.")
    rospy.spin()

if __name__ == "__main__":
    my_depth_server()