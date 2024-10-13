from skiros2_skill.core.skill import SkillDescription, ParamOptions, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import skiros2_common.tools.logger as log
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from skiros2_std_reasoners.aau_spatial_reasoner import AauSpatialReasoner
import rospy
import threading
import tf2_ros as tf

class Navigation:
    def __init__(self):
        """
        TASK:
            Set up all the variables etc. you need in order to perform navigation tasks.
        """

        # Create an action client to connect to the move_base server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Wait for the move_base server to start
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        pass
        

    """
    TASK (OPTIONAL):
        Add more methods if needed.
    """

    def drive_to(self, pose, max_tries=10, sleep_seconds_between_tries=5.0):
        """
        TASK:
            Implement this function.
            Given a ROS Pose message, this function should make the robot drive to that location.

        TASK (OPTIONAL):
            Make the robot retry if it fails.
            In this case you should make use of the max_tries variable.
            Between each attempt, you should wait a number of seconds specified by
            the sleep_seconds_between_tries variable.

        Parameters:
            pose: A ROS Pose message object.
            max_tries: An integer describing the maximum number of driving attempts.
            sleep_seconds_between_tries: A float describing how long to sleep between attempts (in seconds).
        Returns:
            A boolean representing success/fail.
        """

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Target pose is in the map coordinate frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        for attempt in range(max_tries):
            rospy.loginfo(f"Attempt {attempt + 1} to drive to target.")

            # Send the navigation goal
            self.client.send_goal(goal)

            # Wait for the result
            finished_within_time = self.client.wait_for_result(rospy.Duration(60))  # Wait for 60 seconds
            if not finished_within_time:
                rospy.logwarn("Timed out waiting for result. Canceling goal.")
                self.client.cancel_goal()
            else:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached the target successfully.")
                    return True  # Navigation succeeded
                else:
                    rospy.logwarn(f"Failed to reach the target. Current state: {state}")

            # Sleep before retrying
            rospy.sleep(sleep_seconds_between_tries)

        rospy.logerr("Failed to reach the target after maximum attempts.")
        return False  # Navigation failed


class Drive(SkillDescription):   
    def createDescription(self):
        # =======Params=========
        self.addParam("TargetLocation", Element("scalable:Workstation"),ParamTypes.Required)   

class drive(PrimitiveBase):

    def createDescription(self):
        self.setDescription(Drive(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        navigation = Navigation()

        target_location = self.params["TargetLocation"].value

        reasoner: AauSpatialReasoner = self.wmi.get_reasoner("AauSpatialReasoner")
        tl: tf.TransformListener = reasoner._tl

        reasoner.transform(target_location, "map")
        pose = target_location.getData(":PoseMsg")

        self.result = navigation.drive_to(pose)
        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)
