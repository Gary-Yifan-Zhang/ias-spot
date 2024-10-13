#!/usr/bin/env python3

'''
Author: DexterZzz1010 DexterZzz1010@gmail.com
Date: 2024-10-07 08:30:25
LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
LastEditTime: 2024-10-08 11:18:02
FilePath: /catkin_ws/src/Ias-Skills-Handout/nodes/perception_action_client_test.py
Description: vivo50

Copyright (c) 2024 by DexterZzz1010@gmail.com, All Rights Reserved. 
'''

import rospy
import actionlib
from ias_skills_msgs.msg import EstimatePoseAction, EstimatePoseGoal

class ButtonPerceptionClient:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('button_perception_client')

        # 创建一个ActionClient来与ActionServer通信
        self.client = actionlib.SimpleActionClient('button_perception', EstimatePoseAction)

        # 等待ActionServer启动（超时时间为10秒）
        rospy.loginfo("Waiting for the action server to start...")
        if not self.client.wait_for_server(rospy.Duration(100)):
            rospy.logerr("Action server not available!")

    def send_goal(self, object_prompt):
        """
        发送按钮检测请求
        :param object_prompt: 要检测的物体提示，默认为“Detect the red coke can on the table”
        """
        # 定义目标消息
        goal = EstimatePoseGoal()
        goal.object_prompt = object_prompt  # 根据实际需要设置

        # 向ActionServer发送目标
        rospy.loginfo(f"Sending button perception request with prompt: {object_prompt}")
        self.client.send_goal(goal)

    def get_result(self):
        """
        等待结果并返回
        """
        # 等待结果（超时时间为300秒）
        self.client.wait_for_result(rospy.Duration(3000))

        # 获取结果
        result = self.client.get_result()

        if result:
            rospy.loginfo("Button perception successful!")
            rospy.loginfo(f"Detected Button Pose:\n"
                          f"Position: x={result.estimated_pose.pose.position.x}, "
                          f"y={result.estimated_pose.pose.position.y}, "
                          f"z={result.estimated_pose.pose.position.z}\n"
                          f"Orientation (quaternion): x={result.estimated_pose.pose.orientation.x}, "
                          f"y={result.estimated_pose.pose.orientation.y}, "
                          f"z={result.estimated_pose.pose.orientation.z}, "
                          f"w={result.estimated_pose.pose.orientation.w}")
        else:
            rospy.logerr("Button perception failed or no result received.")

if __name__ == '__main__':
    try:
        # 创建客户端对象
        button_client = ButtonPerceptionClient()

        # 发送目标检测请求
        button_client.send_goal("red coke can")  # 可根据需要修改提示词

        # 获取结果
        button_client.get_result()
    except rospy.ROSInterruptException:
        pass