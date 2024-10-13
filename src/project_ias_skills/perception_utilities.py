from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
import skiros2_common.tools.logger as log
import sys
import threading
import math

import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf
import cv2 as cv
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import pyrealsense2
from typing import Tuple
import numpy.linalg as la

class PointCloudListener:
    def __init__(self, topic='/realsense/depth/points'):
        self.topic = topic
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.point_cloud: PointCloud2 = None
    
    def callback(self, data: PointCloud2):
        if self.point_cloud is not None: 
            return

        self.point_cloud = data
    
    def get(self)->Tuple[np.ndarray,str]:
        self.point_cloud = None
        while self.point_cloud is None:
            self.rate.sleep()

        points = point_cloud2.read_points(self.point_cloud)
        points = np.array(list(points))
        points_3d = points[:,0:3]
        return points_3d, self.point_cloud.header.frame_id

    def __enter__(self):
        self.sub = rospy.Subscriber(self.topic, PointCloud2, callback=self.callback)
        return self

    def __exit__(self, type, value, traceback):
        self.sub.unregister()
        del self.sub


class DepthListener:
    # def __init__(self, topic='/realsense/aligned_depth_to_color/image_raw'):
    def __init__(self, topic='/realsense/depth/image_raw'):
        self.topic = topic
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image: np.ndarray = None

    def callback(self, data: Image):
        if self.image is not None: return

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
        except:
            pass

    def __enter__(self)->"DepthListener":
        self.sub = rospy.Subscriber(self.topic, Image, callback=self.callback)
        return self

    def __exit__(self, type, value, traceback):
        self.sub.unregister()
        del self.sub

    def get(self):
        rospy.loginfo("Gettig Depth Image ")
        self.image = None
        while self.image is None:
            self.rate.sleep()
            rospy.loginfo("Depth Image None")
        return self.image


class RGBListener:
    def __init__(self, topic='/realsense/rgb/image_raw'):
        self.topic = topic
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image: np.ndarray = None

    def callback(self, data: Image):
        if self.image is not None: return

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            rospy.loginfo("Image successfully converted")
        except:
            pass
        
        # try:
        #     self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #     rospy.loginfo("Image successfully converted")
        # except CvBridgeError as e:
        #     rospy.logerr(f"Failed to convert image: {e}")
        #     self.image = None

    def __enter__(self)->"RGBListener":
        self.sub = rospy.Subscriber(self.topic, Image, callback=self.callback)
        return self

    def __exit__(self, type, value, traceback):
        self.sub.unregister()
        del self.sub

    def get(self):
        rospy.loginfo("Gettig RGB Image ")
        self.image = None
        while self.image is None:
            self.rate.sleep()
            rospy.loginfo("Image None")
        return self.image

    # def get(self):
    #     self.image = None
    #     while self.image is None:
    #         self.rate.sleep()
    #         if rospy.Time.now().to_sec() - start_time > timeout:
    #             rospy.logerr("Timeout waiting for image")
    #             return None
    #     return self.image


class CameraPoseEstimator:
    @staticmethod
    def receive_camera_info(info_topic):
        class Dummy:
            def __init__(self):
                self.data: CameraInfo = None
            def callback(self, x):
                self.data = x

        data = Dummy()
        sub = rospy.Subscriber(info_topic, CameraInfo, callback=data.callback)

        r = rospy.Rate(10)
        while data.data is None: r.sleep()
        sub.unregister()

        k = np.array(data.data.K).reshape((3,3))
        coeffs = data.data.D
        dist_model = data.data.distortion_model
        w, h = data.data.width, data.data.height
        fx, fy = k[0,0], k[1,1]
        cx, cy = k[0,2], k[1,2]

        return (fx, fy), (cx, cy), (w,h), k, coeffs, dist_model

    def __init__(self, info_topic):
        (self.fx, self.fy), (self.cx, self.cy), (self.w, self.h), self.k, self.coeffs, self.dist_model = self.receive_camera_info(info_topic)

        intrs = pyrealsense2.pyrealsense2.intrinsics()
        intrs.fx, intrs.fy = self.fx, self.fy
        intrs.ppx, intrs.ppy = self.cx, self.cy
        intrs.width, intrs.height = self.w, self.h
        intrs.coeffs = self.coeffs

        assert self.dist_model == "plumb_bob", "unsupported dist model: %s" % self.dist_model
        intrs.model = self.dist_model = pyrealsense2.pyrealsense2.distortion.none

        self.intrs = intrs

    def pixel_to_3d(self, pix_x, pix_y, depth_m):
        """deprecated"""
        x_3d = (pix_x - self.cx) * depth_m / self.fx
        y_3d = (pix_y - self.cy) * depth_m / self.fy
        return x_3d, y_3d
    
    def pixel_to_point(self, pix_x, pix_y, depth):
        return tuple(pyrealsense2.rs2_deproject_pixel_to_point(self.intrs, [pix_x, pix_y], depth))

    def point_to_pixel(self, points: np.ndarray)->np.ndarray:
        output = np.zeros(shape=(points.shape[0],2))
        for i in range(points.shape[0]):
            output[i] = pyrealsense2.rs2_project_point_to_pixel(self.intrs, points[i])
            
        return output

class Transformer:
    def __init__(self, tflistener: tf.listener.TransformListener):
        self.tflistener = tflistener

    def transform_coordinates(self, x, y, z, from_frame, to_frame):
        stamped = PoseStamped()
        stamped.pose.position.x = x
        stamped.pose.position.y = y
        stamped.pose.position.z = z
        stamped.header.frame_id = from_frame

        timestamp = rospy.Time(0)
        stamped.header.stamp = timestamp

        self.tflistener.waitForTransform(to_frame, from_frame, timestamp, rospy.Duration(5))
        transformed = self.tflistener.transformPose(to_frame, stamped)

        res_x = transformed.pose.position.x
        res_y = transformed.pose.position.y
        res_z = transformed.pose.position.z
        return res_x, res_y, res_z

def dist_to_plane(plane: np.ndarray, points: np.ndarray)->np.ndarray:
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_plane    
    """计算点到平面的距离。

    参数：
        plane (np.ndarray): 平面方程的系数 [a, b, c, d]，满足 ax + by + cz + d = 0。
        points (np.ndarray): 形状为 (N, 3) 的点云数据。

    返回：
        distances (np.ndarray): 每个点到平面的距离数组。
    """
    a, b, c, d = plane[0], plane[1], plane[2], plane[3]
    x, y, z = points[:,0], points[:,1], points[:,2]

    mag = math.sqrt(a**2 + b**2 + c**2)
    return np.abs(a * x + b * y + c * z - d) / mag

def svd(A):
    u, s, vh = la.svd(A)
    S = np.zeros(A.shape)
    S[:s.shape[0], :s.shape[0]] = np.diag(s)
    return u, S, vh

def fit_plane_LSE(points):
    # points: Nx4 homogeneous 3d points
    # return: 1d array of four elements [a, b, c, d] of
    # ax+by+cz+d = 0
    """使用最小二乘法拟合平面。

    参数：
        points (np.ndarray): 齐次坐标形式的点集，形状为 (N, 4)，最后一列为 1。

    返回：
        null_space (np.ndarray): 平面方程的系数 [a, b, c, d]，满足 ax + by + cz + d = 0。
    """
    assert points.shape[0] >= 3 # at least 3 points needed
    U, S, Vt = svd(points)
    null_space = Vt[-1, :]
    return null_space

def get_point_dist(points, plane):
    # return: 1d array of size N (number of points)
    dists = np.abs(points @ plane) / np.sqrt(plane[0]**2 + plane[1]**2 + plane[2]**2)
    return dists