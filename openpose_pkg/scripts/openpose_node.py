#!/usr/bin/env python

# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
from sys import platform

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from openpose_pkg.srv import MyDepth

import argparse
import time

# Import Openpose (Ubuntu/OSX)
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath('/home/openpose/build/python'))
from openpose import pyopenpose as op


class OpenPoseNode():
    def __init__(self, params, no_display):
        rospy.init_node('openpose_node')

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()
        self.no_display = no_display
        rospy.loginfo('OpenPose predictor is ready')

        # self.predictor = Predictor(weights_path)
        self.bridge = CvBridge()
        self.img_subscriber = rospy.Subscriber('pose_detection_images', Image, self.predict_pose, queue_size=10)
        self.det_publisher = rospy.Publisher('detection', String, queue_size=10)
        self.depth_client = rospy.ServiceProxy('depth_matrix', MyDepth)
        try:
            resp = self.depth_client('send matrix!')
        except:
            pass
            # print("Service call failed: %s"%e)

        rospy.loginfo('OpenPose node is done')

    def calculate_3d(self, px_point):
        depth_msg = self.depth_client('send matrix!')
        depth = depth_msg.an_integer
        K = [
            [1, 0, 2],
            [0, 3, 4],
            [0, 0, 1]
            ]
        cx, cy = K[0][2], K[1][2]
        fx, fy = K[0][0], K[1][1]
        x = depth*(px_point[0] - cx)/fx
        y = depth*(px_point[1] - cy)/fy
        z = depth

        return x, y, z


    def predict_pose(self, image_msg: Image):
        rospy.loginfo('Received image')
        current_time = time.time()
        image = self.bridge.imgmsg_to_cv2(image_msg, 'rgb8')
        # image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        # current_time = time.time()
        
        datum = op.Datum()
        # imageToProcess = cv2.imread(imagePath)
        datum.cvInputData = image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        if not self.no_display:
            cv2.imshow("OpenPose 1.7.0", datum.cvOutputData)
            key = cv2.waitKey(15)
            # if key == 27: break

        keypoints = datum.poseKeypoints
        n_people = keypoints.shape[0]

        for kp in keypoints:
            left_hand_px = (kp[4][0], kp[4][1])
            right_hand_px =  (kp[7][0], kp[7][1])

            left_hand_3d = self.calculate_3d(left_hand_px)
            right_hand_3d = self.calculate_3d(right_hand_px)

            print(f'right hand : {right_hand_3d}\n',
                  f'left hand  : {left_hand_3d}')

        passed_time = time.time() - current_time
        rospy.loginfo(f'Detected {n_people} people, this took {passed_time} seconds')
        self.det_publisher.publish(f'Detected {n_people} people')


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--image_dir", default="/home/openpose/examples/media/", 
                                help="Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).")
        parser.add_argument("--no_display", default=False, 
                                help="Enable to disable the visual display.")
        args = parser.parse_known_args()

        params = dict()
        params["model_folder"] = "/home/openpose/models/"
        # params["net_resolution"] = "-512x256"
        params["net_resolution"] = "-128x64"

        for i in range(0, len(args[1])):
            curr_item = args[1][i]
            if i != len(args[1])-1: next_item = args[1][i+1]
            else: next_item = "1"
            if "--" in curr_item and "--" in next_item:
                key = curr_item.replace('-','')
                if key not in params:  params[key] = "1"
            elif "--" in curr_item and "--" not in next_item:
                key = curr_item.replace('-','')
                if key not in params: params[key] = next_item

        openpose_node = OpenPoseNode(params, args[0].no_display)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
