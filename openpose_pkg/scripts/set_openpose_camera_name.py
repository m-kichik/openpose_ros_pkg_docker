#!/usr/bin/env python

import argparse

import rospy
from openpose_pkg.srv import SetCam

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--camera_name', default='zednode',
                help='define name of camera topic for following cameras: zednode, gripper or back')
        args = parser.parse_known_args()

        camera_name = args[0].camera_name

        rospy.wait_for_service('openpose/set_camera_name_service')
        set_cam_serv = rospy.ServiceProxy('openpose/set_camera_name_service', SetCam)
        responce = set_cam_serv(camera_name)

        print(responce.responce)

    except rospy.ROSInterruptException:
        pass