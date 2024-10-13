#!/usr/bin/env python3

'''
Author: DexterZzz1010 DexterZzz1010@gmail.com
Date: 2024-10-07 07:53:57
LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
LastEditTime: 2024-10-10 08:10:18
FilePath: /catkin_ws/src/Ias-Skills-Handout/nodes/button_perception_action_server.py
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
from sklearn.linear_model import RANSACRegressor
from project_ias_skills.perception_utilities import *
from robotlablth.imai_client import Client  # Client for object detection service
from geometry_msgs.msg import PoseStamped
from tf import TransformListener
from tf.transformations import quaternion_from_matrix, quaternion_from_euler
import tf
from project_ias_skills.transform_utils import orthonormalize_rotation  # For orthonormalizing rotation matrices


class ButtonPerceptionServer(object):
    _feedback = EstimatePoseFeedback()
    _result = EstimatePoseResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, EstimatePoseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.tf_listener = TransformListener()
        
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
    
    def show_detect_region(self, rgb_image, detection):
        # Create a copy of the image to avoid directly modifying the original image
        img_copy = np.copy(rgb_image)

        # Get the list of detection regions (bounding boxes)
        regions = detection.regions
        x_min, y_min, x_max, y_max = regions[0]

        # Draw a bounding box on the image, with red color and a thickness of 2
        cv2.rectangle(img_copy, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

        # Display the annotated image
        cv2.imshow('Detected Region', img_copy)
        cv2.waitKey(0)  # Press any key to close the window
        cv2.destroyAllWindows()

    def compute_pca_frame(self, points_3d):
        """
        Perform PCA on the 3D point cloud and return a local 3D coordinate system (frame)

        Parameters:
            points_3d (numpy.ndarray): Point cloud data with shape (N, 3), where N is the number of points, and 3 represents the x, y, z coordinates.
            
        Returns:
            centroid (numpy.ndarray): The centroid coordinates of the point cloud (3,)
            pca_components (numpy.ndarray): A matrix (3, 3) composed of the three orthogonal principal components.
        """
        # Compute the centroid
        centroid = np.mean(points_3d, axis=0)

        # Center the point cloud
        centered_points = points_3d - centroid

        # Perform PCA
        pca = PCA(n_components=3)
        pca.fit(centered_points)

        # Principal components of PCA
        X_axis = pca.components_[0]  # Principal X direction
        Y_axis = pca.components_[1]  # Secondary Y direction
        Z_axis = pca.components_[2]  # Secondary Z direction

        # Construct the rotation matrix
        rotation_matrix = np.vstack([X_axis, Y_axis, Z_axis]).T
        
        # Convert the rotation matrix to quaternion
        quaternion = quaternion_from_matrix(
            np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1]))
        )

        return centroid, quaternion

    def fit_plane_ransac(self, points_3d):
        """
        Use RANSAC to fit a plane to the point cloud and extract the plane's normal as the Z-axis direction.
        """
        # Extract x and y as independent variables, and z as the dependent variable
        X = points_3d[:, :2]  # Use x, y data as independent variables
        y = points_3d[:, 2]   # Use z data as the dependent variable

        # Use RANSAC for plane fitting
        ransac = RANSACRegressor(estimator=LinearRegression(), min_samples=50, residual_threshold=0.01)
        ransac.fit(X, y)

        # Plane parameters ax + by + c = z
        a, b = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_

        # The normal direction of the plane is [a, b, -1]
        normal = np.array([a, b, -1])
        normal /= np.linalg.norm(normal)  # Normalize

        return normal, (a, b, c)

    def compute_plane_orientation(self, normal):
        """
        Use the plane's normal as the Z-axis direction, construct a rotation matrix, and convert it to a quaternion.
        """
        # Assume the Z-axis is the normal
        Z_axis = normal

        # Assume the X-axis is [1, 0, 0], and compute the Y-axis using the cross product
        X_axis = np.array([1, 0, 0])

        # If Z-axis and X-axis are collinear (rare), use [0, 1, 0] as the X-axis
        if np.allclose(np.abs(Z_axis), np.abs(X_axis)):
            X_axis = np.array([0, 1, 0])

        # Compute the Y-axis using the cross product
        Y_axis = np.cross(Z_axis, X_axis)
        Y_axis /= np.linalg.norm(Y_axis)  # Normalize

        # Recompute the X-axis to ensure orthogonality
        X_axis = np.cross(Y_axis, Z_axis)
        X_axis /= np.linalg.norm(X_axis)

        # Construct the rotation matrix
        rotation_matrix = np.vstack([X_axis, Y_axis, Z_axis]).T

        # Convert the rotation matrix to quaternion
        quaternion = quaternion_from_matrix(
            np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1]))
        )

        return quaternion

    def execute_cb(self, goal: EstimatePoseGoal):
        text_prompt = goal.object_prompt
        rospy.loginfo("Starting button perception using prompt '%s'", text_prompt)

        rgb_listener = RGBListener()
        depth_listener = DepthListener()
        pts_listener = PointCloudListener()

        transformer = Transformer(self.tf_listener)
        client = Client("localhost:50051", token="test")

        with rgb_listener, depth_listener, pts_listener, client:
            self.status("Capturing RGB and depth images...")

            # Delay
            rospy.sleep(5)
            
            if self.should_preempt():
                return

            # Get RGB and depth images
            rgb_image = rgb_listener.get()
            depth_image = depth_listener.get()

            # Check if RGB image is received and log it
            if rgb_image is not None:
                rospy.loginfo("Successfully received RGB image")

            # Check if Depth image is received and log it
            if depth_image is not None:
                rospy.loginfo("Successfully received Depth image")
      
            self.status("Capturing 3D point cloud...")
            # Get 3D point cloud
            points_3d, frame_id = pts_listener.get()

            self.status("Performing button detection and segmentation...")
            if self.should_preempt():
                return

            # Call the detection model to get the detection result
            text_obj = client.text_prompt_object_detection(text_prompt, rgb_image)

            if not text_obj or not text_obj.regions.size:
                rospy.logerr("No objects detected with the prompt '%s'", text_prompt)
                self._as.set_aborted()
                return
            
            self.show_detect_region(rgb_image, text_obj)

            # Extract the detected region (regions are in the format [y1, x1, y2, x2])
            detection_region = text_obj.regions[0]
            y1, x1, y2, x2 = detection_region  # Extract the coordinates of the detected region
            rospy.loginfo(f"Detected region: {detection_region}, with confidence {text_obj.scores[0]}")

            self.status("Extracting button point cloud...")
            if self.should_preempt():
                return

            # Assume points_3d is an array of shape (H, W, 3) matching the image dimensions
            H, W = rgb_image.shape[:2]
            points_3d = points_3d.reshape((H, W, 3))

            # Extract the point cloud of the detected object (select only points within the region)
            object_points = points_3d[y1:y2, x1:x2].reshape(-1, 3)

            # Filter out invalid points (e.g., NaN or infinite values)
            valid_indices = np.isfinite(object_points[:, 2])
            object_points = object_points[valid_indices]

            if len(object_points) == 0:
                rospy.logerr("No valid points found for the detected object.")
                self._as.set_aborted()
                return

            self.status("Computing button pose using PCA and fitting a plane...")
            if self.should_preempt():
                return

            # Compute the center of mass (centroid)
            centroid = np.mean(object_points, axis=0)

            # ------------------------------------------------------------------------------------------ #
            '''
            I'm not sure if the code below is useful,
            because the orientation of the button is always straightforward.
            '''
            # PCA
            # centroid, rotation_quat = self.compute_pca_frame(object_points)

            # RANSAC plane fitting
            normal, plane_params = self.fit_plane_ransac(object_points)

            # Compute the plane's quaternion orientation
            rotation_quat = self.compute_plane_orientation(normal)

            # ------------------------------------------------------------------------------------------ #

            self.status("Transforming button pose into base frame...")
            if self.should_preempt():
                return

            try:
                self.tf_listener.waitForTransform('ur5e_base_link', 'realsense_rgb_optical_frame', rospy.Time(0), rospy.Duration(4.0))
                x_world, y_world, z_world = transformer.transform_coordinates(
                    centroid[0], centroid[1], centroid[2], from_frame=frame_id, to_frame='ur5e_base_link'
                )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("TF transformation failed: %s", e)
                self._as.set_aborted()
                return

            # Set the result
            self._result.estimated_pose.header.frame_id = 'ur5e_base_link'
            self._result.estimated_pose.header.stamp = rospy.Time.now()

            self._result.estimated_pose.pose.position.x = x_world
            self._result.estimated_pose.pose.position.y = y_world
            self._result.estimated_pose.pose.position.z = z_world

            self._result.estimated_pose.pose.orientation.x = rotation_quat[0]
            self._result.estimated_pose.pose.orientation.y = rotation_quat[1]
            self._result.estimated_pose.pose.orientation.z = rotation_quat[2]
            self._result.estimated_pose.pose.orientation.w = rotation_quat[3]

            # Assume that the posture of the new building block is consistent with the normal direction of the desktop
            # self._result.estimated_pose.pose.orientation.x = 0.0
            # self._result.estimated_pose.pose.orientation.y = 0.0
            # self._result.estimated_pose.pose.orientation.z = 0.0
            # self._result.estimated_pose.pose.orientation.w = 1.0

            self._as.set_succeeded(self._result)

            rospy.loginfo("Button Perception successful.")


if __name__ == '__main__':
    rospy.init_node('button_perception')
    server = ButtonPerceptionServer("button_perception")
    rospy.spin()
