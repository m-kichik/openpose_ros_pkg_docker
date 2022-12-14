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
from openpose_pkg.srv import SetCam

import argparse
import time

# Import Openpose (Ubuntu/OSX)
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath('/home/openpose/build/python'))
from openpose import pyopenpose as op


class OpenPoseNode():
    def __init__(self, params, camera_name='zednode'):
        rospy.init_node('openpose_node')

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()

        rospy.loginfo('OpenPose predictor is ready')

        self.points_dict = {
                    0 : "Nose",
                    1 : "Neck",
                    2 : "RShoulder",
                    3 : "RElbow",
                    4 : "RWrist",
                    5 : "LShoulder",
                    6 : "LElbow",
                    7 : "LWrist",
                    8 : "MidHip",
                    9 : "RHip",
                    10: "RKnee",
                    11: "RAnkle",
                    12: "LHip",
                    13: "LKnee",
                    14: "LAnkle",
                    15: "REye",
                    16: "LEye",
                    17: "REar",
                    18: "LEar",
                    19: "LBigToe",
                    20: "LSmallToe",
                    21: "LHeel",
                    22: "RBigToe",
                    23: "RSmallToe",
                    24: "RHeel",

                }

        self.camera_name = None
        self.important_points = None
        self.distance_1 = None
        self.distance_2 = None

        self.set_camera(camera_name)

        self.bridge = CvBridge()

        # init subscribers for all camera topics:
        self.init_subscribers()

        # init publishers:
        self.init_publishers()

        # init service for setting camera
        self.set_cam_service = rospy.Service('openpose/set_camera_name_service', SetCam, self.set_camera)

        rospy.loginfo('OpenPose node is done')


    def set_camera(self, camera_name_msg):
        if type(camera_name_msg) == str:
            camera_name = camera_name_msg
        else:
            camera_name = camera_name_msg.camera_name

        rospy.loginfo(f'Set {camera_name} node name')
        self.camera_name = camera_name

        important_points = {
            'zednode': [0, 1, 2, 5, 15, 16, 17, 18],
            'back': [3, 4, 6, 7, 8, 9, 12],
            'gripper': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15, 16, 17, 18],
        }

        self.important_points = important_points[camera_name]

        # d_1 - for one human who is too close
        # d_2 - for two and more people when all of them are too close

        distance_1 = {
            'zednode': 1.,
            'back': 1000.,
            'gripper': 500,
        }

        distance_2 = {
            'zednode': 2.,
            'back': 1500.,
            'gripper': 1000,
        }

        self.distance_1 = distance_1[camera_name]
        self.distance_2 = distance_2[camera_name]

        return f'Successfully set {camera_name} camera name'

    
    def init_subscribers(self):
        # zednode:

        zednode_img_subscriber = message_filters.Subscriber(
            '/zed_node/left/image_rect_color/compressed', Image
            )

        zednode_depth_subscriber = message_filters.Subscriber(
            '/zed_node/depth/depth_registered', Image
            )

        zednode_depth_info_subscriber = message_filters.Subscriber(
            '/zed_node/depth/camera_info', CameraInfo
            )

        zednode_syncronizer = message_filters.TimeSynchronizer(
            [zednode_img_subscriber, zednode_depth_subscriber, zednode_depth_info_subscriber], 10
            )

        zednode_syncronizer.registerCallback(self.zednode_do_detection)

        # gripper:

        gripper_img_subscriber = message_filters.Subscriber(
            '/realsense_gripper/color/image_raw', Image
            )

        gripper_depth_subscriber = message_filters.Subscriber(
            '/realsense_gripper/aligned_depth_to_color/image_raw', Image
            )

        gripper_depth_info_subscriber = message_filters.Subscriber(
            '/realsense_gripper/depth/camera_info', CameraInfo
            )

        gripper_syncronizer = message_filters.TimeSynchronizer(
            [gripper_img_subscriber, gripper_depth_subscriber, gripper_depth_info_subscriber], 10
            )

        gripper_syncronizer.registerCallback(self.gripper_do_detection)

        # back:

        back_img_subscriber = message_filters.Subscriber(
            '/realsense_back/color/image_raw', Image
            )

        back_depth_subscriber = message_filters.Subscriber(
            '/realsense_back/aligned_depth_to_color/image_raw', Image
            )

        back_depth_info_subscriber = message_filters.Subscriber(
            '/realsense_back/depth/camera_info', CameraInfo
            )

        back_syncronizer = message_filters.TimeSynchronizer(
            [back_img_subscriber, back_depth_subscriber, back_depth_info_subscriber], 10
            )

        back_syncronizer.registerCallback(self.back_do_detection)


    def init_publishers(self):
        self.det_publisher = rospy.Publisher('openpose/detection', String, queue_size=10)

        self.det_img_publisher = rospy.Publisher('openpose/img_detection', Image, queue_size=10)

        self.close_people_publisher = rospy.Publisher('openpose/close_people', String, queue_size=10)
    
    
    def zednode_do_detection(self, image_msg: Image, depth_msg: Image, depth_info_msg: CameraInfo):
        if self.camera_name == 'zednode':
            self.predict_pose(image_msg, depth_msg, depth_info_msg)


    def gripper_do_detection(self, image_msg: Image, depth_msg: Image, depth_info_msg: CameraInfo):
        if self.camera_name == 'gripper':
            self.predict_pose(image_msg, depth_msg, depth_info_msg)

    
    def back_do_detection(self, image_msg: Image, depth_msg: Image, depth_info_msg: CameraInfo):
        if self.camera_name == 'back':
            self.predict_pose(image_msg, depth_msg, depth_info_msg)

    
    def calculate_3d(self, px_point, depth, K):

        px_x, px_y = px_point

        x_size = depth.shape[1]
        y_size = depth.shape[0]

        if 0 <= px_x <= x_size and 0 <= px_y <= y_size:
            depth_p = depth[int(px_y)][int(px_x)]

            if not np.isnan(depth_p):
                cx, cy = K[2], K[5]
                fx, fy = K[0], K[4]
                x = depth_p*(px_point[0] - cx)/fx
                y = depth_p*(px_point[1] - cy)/fy
                z = depth_p

                return x, y, z
            else:
                return 0, 0, 0
        else:
            return 0, 0, 0


    def predict_pose(self, image_msg: Image, depth_msg: Image, depth_info_msg: CameraInfo):
        rospy.loginfo('Received image')
        current_time = time.time()

        image = self.bridge.imgmsg_to_cv2(image_msg, 'rgb8')
        # image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        # np_arr = np.fromstring(image_msg.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        datum = op.Datum()
        datum.cvInputData = image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        depth = self.bridge.imgmsg_to_cv2(depth_msg)

        keypoints = datum.poseKeypoints

        # what happens in the following part:
        # for each person for each point we calculate its 3d coordinates
        # then we filter z-coordinate for "important points" - head and shoulders,
        # and when someone is closer to the camera than max(d1, d2) we add his (maybe her) 
        # average depth to the list "close people dists";
        # then we find out if there are more than one person in "close peoples" and publish code 2;
        # if there was only one person compare his (or her) distance with d1 and maybe publish code 1;
        # if nobody is too close we publish code 0.

        keypoints_3d = []

        close_people_dists = []

        if keypoints is not None:
            n_people = keypoints.shape[0]

            for kp in keypoints:

                pose_3d = dict.fromkeys(self.points_dict.keys())

                for k in self.points_dict.keys():
                    pose_3d[k] = self.calculate_3d(
                        (kp[k][0], kp[k][1]), depth, depth_info_msg.K
                    )

                keypoints_3d.append(pose_3d)

                important_depths = [pose_3d[x][2] for x in self.important_points]
                important_depths = [x for x in important_depths if x > 0]
                sum_depth = sum(important_depths)

                if sum_depth != 0:
                    av_depth = sum_depth / len(important_depths)
                else:
                    av_depth = 1000

                if av_depth <= max(self.distance_1, self.distance_2):
                    close_people_dists.append(av_depth)

        else:
            n_people = 0

        if len(close_people_dists) > 1:
            self.close_people_publisher.publish(f'{self.camera_name[0]}2')
        else:
            if len(close_people_dists) == 1:
                if close_people_dists[0] <= self.distance_1:
                    self.close_people_publisher.publish(f'{self.camera_name[0]}1')
            else:
                self.close_people_publisher.publish('no_close')

        # passed_time = time.time() - current_time
        self.det_img_publisher.publish(self.bridge.cv2_to_imgmsg(datum.cvOutputData, 'rgb8'))
        self.det_publisher.publish(f'Detected {n_people} people')
        passed_time = time.time() - current_time
        rospy.loginfo(f'Detected {n_people} people, this took {passed_time} seconds')


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--net_resolution', default='256x128',
                help='define openpose net resolution, each of the numbers must be divisible by 16')
        parser.add_argument('--topic', default='zednode',
                help='define name of camera topic for following cameras: zednode, gripper or back')
        args = parser.parse_known_args()

        params = dict()
        params["model_folder"] = "/home/openpose/models/"
        params["net_resolution"] = args[0].net_resolution

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
