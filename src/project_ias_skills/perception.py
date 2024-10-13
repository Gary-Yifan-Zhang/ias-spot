#!/usr/bin/env python
'''
Author: DexterZzz1010 DexterZzz1010@gmail.com
Date: 2024-10-03 15:41:56
LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
LastEditTime: 2024-10-10 08:50:39
FilePath: /catkin_ws/src/Ias-Skills-Handout/src/project_ias_skills/perception.py
Description: vivo50

Copyright (c) 2024 by DexterZzz1010@gmail.com, All Rights Reserved. 
'''
import rospy
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import actionlib
from ias_skills_msgs.msg import EstimatePoseAction, EstimatePoseGoal
from actionlib_msgs.msg import GoalStatus
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import threading
from .perception_utilities import *

try:
    import Queue as queue
except ImportError:
    import queue


#################################################################################
# Button Perception Skill Description
#################################################################################

class DetectButton(SkillDescription):
    """
    Define the parameters for button perception.
    """

    def createDescription(self):
        self.addParam("Object", Element("skiros:TransformationPose"), ParamTypes.Required,
                      description="The position of the prompt object")


#################################################################################
# Button Perception Skill Implementation
#################################################################################

class detect_button(PrimitiveActionClient):
    """
    Button perception skill for detecting the position and orientation of a button
    based on a prompt.
    """

    def createDescription(self):
        """
        Set the description for the skill.
        """
        self.setDescription(DetectButton(), self.__class__.__name__)

    def buildClient(self):
        """
        Create the action client to communicate with the button perception action server.
        """
        return actionlib.SimpleActionClient('button_perception', EstimatePoseAction)

    def buildGoal(self):
        goal = EstimatePoseGoal()
        goal.object_prompt = "red coke can"

        rospy.loginfo(f"Sending button detection request with prompt: {goal.object_prompt}")
        return goal

    def onDone(self, status, result):
        """
        Handle the result once the action server completes the goal.
        """
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Button perception successful!")
            rospy.loginfo(f"Detected Button Pose:\n"
                          f"Position: x={result.estimated_pose.pose.position.x}, "
                          f"y={result.estimated_pose.pose.position.y}, "
                          f"z={result.estimated_pose.pose.position.z}\n"
                          f"Orientation (quaternion): x={result.estimated_pose.pose.orientation.x}, "
                          f"y={result.estimated_pose.pose.orientation.y}, "
                          f"z={result.estimated_pose.pose.orientation.z}, "
                          f"w={result.estimated_pose.pose.orientation.w}")

            # Update the world model with the estimated pose
            obj = self.params['Object'].value # defined in the discription
            obj.setData(':PoseStampedMsg', result.estimated_pose)
            self.wmi.update_element_properties(obj)
            # self.done = True
            return self.success("Button pose detected and updated in world model.")
        else:
            rospy.logerr("Button Detection failed.")
            return self.fail("Button Detection failed", -1)

    def run(self):
        """
        The main function that sends the goal to the action server and waits for the result.
        """
        rospy.loginfo("Running Button Detection Task")
        # Wait for result
        self.client.wait_for_result()
        result = self.client.get_result()

        if result:
            self.onDone(self.client.get_state(), result)
        else:
            rospy.logerr("No result received from button perception action server.")
            self.fail("No result received", -1)
        self.done = True
        rospy.loginfo("Button Detection Done")

    def onStart(self):
          # Ensure client is initialized before sending the goal
        if not hasattr(self, 'client'):
            self.client = self.buildClient()  # Explicitly initialize the client if it hasn't been already
        self.fb = queue.Queue(1)
        self.res = queue.Queue(1)
        if self.build_client_onstart:
            self.client = self.buildClient()
        if not self.client.wait_for_server(rospy.Duration(10)):
            return self.startError("Action server {} is not available.".format(self.client.action_client.ns), -101)
        self.client.send_goal(self.buildGoal(), done_cb= self._doneCb, feedback_cb = self._feedbackCb)
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Button Detection Running...')

        self.thread.join()

        return self.success('Button Detection Done')

