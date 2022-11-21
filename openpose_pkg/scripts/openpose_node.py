#!/usr/bin/env python

# command: rosrun openpose_pkg openpose_node.py --net_resolution "384x192" --topic zednode

# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
from sys import platform

import rospy
import message_filters
import cv2
import numpy as np

from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

import argparse
import time

# Import Openpose (Ubuntu/OSX)
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath('/home/openpose/build/python'))
from openpose import pyopenpose as op


class OpenPoseNode():
    def __init__(self, params, topic='zednode'):
        rospy.init_node('openpose_node')

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()
        topic_names = {
            'zednode' : {
                'img'        : '/zed_node/left_raw/image_raw_color',
                'depth'      : '/zed_node/depth/camera_info',
                'depth_info' : '/zed_node/depth/camera_info',
            },
            'gripper' : {
                'img'        : '/realsense_gripper/color/image_raw',
                'depth'      : '/realsense_gripper/aligned_depth_to_color/image_raw',
                'depth_info' : '/realsense_gripper/depth/camera_info',
            },
            'back' : {
                'img'        : '/realsense_back/color/image_raw',
                'depth'      : '/realsense_back/aligned_depth_to_color/image_raw',
                'depth_info' : '/realsense_back/depth/camera_info',
            }
        }
        self.topic = topic_names[topic]

        rospy.loginfo('OpenPose predictor is ready')

        # self.predictor = Predictor(weights_path)
        self.bridge = CvBridge()

        self.img_subscriber = message_filters.Subscriber(
            self.topic['img'], Image
            )

        self.depth_subscriber = message_filters.Subscriber(
            self.topic['depth'], Image
            )

        self.depth_info_subscriber = message_filters.Subscriber(
            self.topic['depth_info'], CameraInfo
            )

        self.syncronizer = message_filters.TimeSynchronizer(
            [self.img_subscriber, self.depth_subscriber, self.depth_info_subscriber], 10
            )

        self.syncronizer.registerCallback(self.predict_pose)

        # self.img_subscriber = rospy.Subscriber(
        #     '/zed_node/left_raw/image_raw_color', Image, 
        #     self.predict_pose, queue_size=10
        #     )

        self.det_publisher = rospy.Publisher('detection', String, queue_size=10)

        self.det_img_publisher = rospy.Publisher('img_detection', Image, queue_size=10)

        rospy.loginfo('OpenPose node is done')

    def calculate_3d(self, px_point, depth, K):

        depth_p = depth[int(px_point[1])][int(px_point[0])]
        cx, cy = K[2], K[5]
        fx, fy = K[0], K[4]
        x = depth_p*(px_point[0] - cx)/fx
        y = depth_p*(px_point[1] - cy)/fy
        z = depth_p

        return x, y, z


    def predict_pose(self, image_msg: Image, depth_msg: Image, depth_info_msg: CameraInfo):
        rospy.loginfo('Received image')
        current_time = time.time()
        image = self.bridge.imgmsg_to_cv2(image_msg, 'rgb8')
        # image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        
        datum = op.Datum()
        datum.cvInputData = image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        depth = self.bridge.imgmsg_to_cv2(depth_msg)

        keypoints = datum.poseKeypoints
        if keypoints is not None:
            n_people = keypoints.shape[0]

            for kp in keypoints:
                left_hand_px = (kp[4][0], kp[4][1])
                right_hand_px =  (kp[7][0], kp[7][1])

                left_hand_3d = self.calculate_3d(left_hand_px, depth, depth_info_msg.K)
                right_hand_3d = self.calculate_3d(right_hand_px, depth, depth_info_msg.K)

                print(f' right hand : {right_hand_3d}\n',
                      f'left hand  : {left_hand_3d}')
        else:
            n_people = 0

        # passed_time = time.time() - current_time
        self.det_img_publisher.publish(self.bridge.cv2_to_imgmsg(datum.cvOutputData, 'rgb8'))
        self.det_publisher.publish(f'Detected {n_people} people')
        passed_time = time.time() - current_time
        rospy.loginfo(f'Detected {n_people} people, this took {passed_time} seconds')


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--topic', default='zednode',
                help='define name of camera topic for following cameras: zednode, gripper or back')
        args = parser.parse_known_args()

        params = dict()
        params["model_folder"] = "/home/openpose/models/"

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

        openpose_node = OpenPoseNode(params, args[0].topic)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
