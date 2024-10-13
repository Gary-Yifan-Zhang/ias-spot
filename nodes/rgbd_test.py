#!/usr/bin/env python
'''
Author: DexterZzz1010 DexterZzz1010@gmail.com
Date: 2024-10-07 11:15:33
LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
LastEditTime: 2024-10-07 14:17:50
FilePath: /catkin_ws/src/Ias-Skills-Handout/nodes/rgbd_test.py
Description: vivo50

Copyright (c) 2024 by DexterZzz1010@gmail.com, All Rights Reserved. 
'''

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageListener:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('image_listener', anonymous=True)

        # CvBridge实例化，用于转换ROS图像消息到OpenCV格式
        self.bridge = CvBridge()

        # 订阅深度图像、RGB图像和相机内参话题
        self.rgb_sub = rospy.Subscriber("/realsense/rgb/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/realsense/depth/image_raw", Image, self.depth_callback)
        # self.depth_sub = rospy.Subscriber("/realsense/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        # self.camera_info_sub = rospy.Subscriber("/realsense/rgb/camera_info", PointCloud2, self.point_cloud_callback)

        self.camera_info_received = False  # 标志位，用于判断是否接收到相机内参
        
        rospy.loginfo("Image listener node initialized.")
    
    def rgb_callback(self, data):
        try:
            # 使用CvBridge将ROS的Image消息转换为OpenCV格式的图像
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 显示RGB图像
            cv2.imshow("RGB Image", rgb_image)
            cv2.waitKey(1)
            
            # 打印图像尺寸信息
            rospy.loginfo("Received RGB image of size: {}x{}".format(rgb_image.shape[1], rgb_image.shape[0]))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def depth_callback(self, data):
        try:
            # 使用CvBridge将ROS的Image消息转换为OpenCV格式的深度图像
            depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

            # 显示深度图像
            cv2.imshow("Depth Image", depth_image)
            cv2.waitKey(1)
            
            # 打印图像尺寸信息
            rospy.loginfo("Received Depth image of size: {}x{}".format(depth_image.shape[1], depth_image.shape[0]))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            

    def camera_info_callback(self, data):
        if not self.camera_info_received:  # 确保只打印一次相机信息
            # 获取相机内参
            fx = data.K[0]
            fy = data.K[4]
            cx = data.K[2]
            cy = data.K[5]
            distortion_coeffs = data.D
            distortion_model = data.distortion_model

            # 打印相机内参和畸变系数
            rospy.loginfo(f"Camera Info Received:\n"
                          f"  Focal Length: fx = {fx}, fy = {fy}\n"
                          f"  Principal Point: cx = {cx}, cy = {cy}\n"
                          f"  Distortion Model: {distortion_model}\n"
                          f"  Distortion Coefficients: {distortion_coeffs}")

            self.camera_info_received = True  # 标志位设为True，避免重复打印

if __name__ == '__main__':
    try:
        # 实例化ImageListener
        listener = ImageListener()

        # 持续运行，直到节点被关闭
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()


