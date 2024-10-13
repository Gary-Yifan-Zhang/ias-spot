#!/usr/bin/env python3

'''
Author: DexterZzz1010 DexterZzz1010@gmail.com
Date: 2024-10-07 13:52:15
LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
LastEditTime: 2024-10-07 15:15:57
FilePath: /catkin_ws/src/Ias-Skills-Handout/nodes/test_perception_action_server.py
Description: vivo50

Copyright (c) 2024 by DexterZzz1010@gmail.com, All Rights Reserved. 
'''

import rospy
from ias_skills_msgs.msg import EstimatePoseAction, EstimatePoseFeedback, EstimatePoseResult, EstimatePoseGoal
import actionlib
import numpy as np
import cv2
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression
from robotlablth.imai_client import Client  # Client for object detection service
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf import TransformListener
from tf.transformations import quaternion_from_matrix
import tf
import sensor_msgs.point_cloud2 as pc2

class ButtonPerceptionServer(object):
    _feedback = EstimatePoseFeedback()
    _result = EstimatePoseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, EstimatePoseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.tf_listener = TransformListener()

        # CvBridge 用于将ROS图像消息转换为OpenCV格式
        self.bridge = CvBridge()

        # 初始化订阅
        self.rgb_image = None
        self.depth_image = None
        self.point_cloud = None

        rospy.Subscriber('/realsense/rgb/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/realsense/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/realsense/depth/points', PointCloud2, self.pointcloud_callback)

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("RGB image conversion failed: %s", e)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("Depth image conversion failed: %s", e)

    def pointcloud_callback(self, msg):
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        self.point_cloud = np.array(points)

    def status(self, text, *args):
        rospy.loginfo(text, *args)
        self._feedback.status = text % tuple(args)
        self._as.publish_feedback(self._feedback)

    def should_preempt(self):
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return True
        return False

    def execute_cb(self, goal: EstimatePoseGoal):
        text_prompt = goal.object_prompt
        rospy.loginfo("Starting button perception using prompt '%s'", text_prompt)

        # client = Client("localhost:50051")

        self.status("Capturing RGB and depth images...")

        # Delay
        rospy.sleep(0.5)

        if self.should_preempt():
            return

        # 检查是否收到了RGB图像、深度图像和点云
        if self.rgb_image is None or self.depth_image is None or self.point_cloud is None:
            rospy.logerr("Failed to capture images or point cloud")
            self._as.set_aborted()
            return

        rospy.loginfo("Successfully received RGB, Depth images and PointCloud")

        self.status("Performing button detection and segmentation...")
        if self.should_preempt():
            return

        # detections = client.text_prompt_object_detection(text_prompt, self.rgb_image)
        # if not detections:
        #     rospy.logerr("No buttons detected with the prompt '%s'", text_prompt)
        #     self._as.set_aborted()
        #     return

        # detection = detections[0]
        # mask = detection.mask

        # self.status("Extracting button point cloud...")
        # if self.should_preempt():
        #     return

        # # 从点云中提取有效点
        # object_points = self.point_cloud[mask > 0]

        # if len(object_points) == 0:
        #     rospy.logerr("No valid points found for the detected object.")
        #     self._as.set_aborted()
        #     return

        # self.status("Computing button pose using PCA and fitting a plane...")
        # if self.should_preempt():
        #     return

        # # 计算质心
        # centroid = np.mean(object_points, axis=0)

        # # 使用PCA和线性回归进行平面拟合
        # pca = PCA(n_components=3)
        # pca.fit(object_points - centroid)
        # X_axis = pca.components_[0]

        # reg = LinearRegression()
        # reg.fit(object_points[:, :2], object_points[:, 2])
        # a, b = reg.coef_
        # Z_axis = np.array([-a, -b, 1])
        # Z_axis /= np.linalg.norm(Z_axis)

        # Y_axis = np.cross(Z_axis, X_axis)
        # Y_axis /= np.linalg.norm(Y_axis)
        # X_axis = np.cross(Y_axis, Z_axis)
        # X_axis /= np.linalg.norm(X_axis)

        # rotation_matrix = np.vstack([X_axis, Y_axis, Z_axis]).T
        # rotation_quat = quaternion_from_matrix(np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1])))

        # self.status("Transforming button pose into base frame...")
        # if self.should_preempt():
        #     return

        # try:
        #     self.tf_listener.waitForTransform('base_link', 'realsense_depth_optical_frame', rospy.Time(0), rospy.Duration(4.0))
        #     x_world, y_world, z_world = self.tf_listener.transformPoint('base_link', centroid)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #     rospy.logerr("TF transformation failed: %s", e)
        #     self._as.set_aborted()
        #     return

        # # 设置估计结果
        # self._result.estimated_pose.header.frame_id = 'base_link'
        # self._result.estimated_pose.header.stamp = rospy.Time.now()
        # self._result.estimated_pose.pose.position.x = x_world
        # self._result.estimated_pose.pose.position.y = y_world
        # self._result.estimated_pose.pose.position.z = z_world
        # self._result.estimated_pose.pose.orientation.x = rotation_quat[0]
        # self._result.estimated_pose.pose.orientation.y = rotation_quat[1]
        # self._result.estimated_pose.pose.orientation.z = rotation_quat[2]
        # self._result.estimated_pose.pose.orientation.w = rotation_quat[3]

        self._as.set_succeeded(self._result)
        rospy.loginfo("Button Perception successful.")

if __name__ == '__main__':
    rospy.init_node('button_perception')
    server = ButtonPerceptionServer("button_perception")
    rospy.spin()
