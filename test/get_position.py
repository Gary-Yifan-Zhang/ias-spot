import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
import cv2
import mediapipe as mp
import numpy as np
from cv_bridge import CvBridge
import pyrealsense2 as rs2
from geometry_msgs.msg import Point
from typing import Tuple

# Initialize MediaPipe Hands and Drawing modules
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class CameraPoseEstimator:
    def __init__(self, info_topic):
        self.bridge = CvBridge()
        self.fx, self.fy, self.cx, self.cy = self.receive_camera_info(info_topic)
    
    def receive_camera_info(self, info_topic):
        msg = rospy.wait_for_message(info_topic, CameraInfo)
        fx = msg.K[0]
        fy = msg.K[4]
        cx = msg.K[2]
        cy = msg.K[5]
        return fx, fy, cx, cy

    def pixel_to_3d(self, x, y, depth):
        """将像素坐标和深度转换为3D空间坐标"""
        X = (x - self.cx) * depth / self.fx
        Y = (y - self.cy) * depth / self.fy
        Z = depth
        return np.array([X, Y, Z])

def recognize_gesture(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    
    # 假设这是检测“Pointing”手势的条件
    if index_finger_tip.y < middle_finger_tip.y:
        return "Pointing", (index_finger_tip.x, index_finger_tip.y)
    return "Other", None

def calculate_palm_center(hand_landmarks, image_width, image_height):
    # 使用几个关键点来近似手掌中心
    palm_landmarks = [
        hand_landmarks.landmark[mp_hands.HandLandmark.WRIST],
        hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC],
        hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP],
        hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
        hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP],
        hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP],
    ]
    avg_x = int(np.mean([landmark.x * image_width for landmark in palm_landmarks]))
    avg_y = int(np.mean([landmark.y * image_height for landmark in palm_landmarks]))
    return avg_x, avg_y

def main():
    rospy.init_node('gesture_detection_node')
    camera_info_topic = "/camera/color/camera_info"
    depth_topic = "/camera/depth/image_rect_raw"

    estimator = CameraPoseEstimator(camera_info_topic)
    bridge = CvBridge()
    
    # 使用MediaPipe的Hands检测手势
    with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7) as hands:
        while not rospy.is_shutdown():
            # 获取深度图像
            depth_msg = rospy.wait_for_message(depth_topic, Image)
            depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            # 读取摄像头图像
            ret, frame = cap.read()
            if not ret:
                continue

            image_height, image_width, _ = frame.shape
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(frame_rgb)

            # 检测手势
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    
                    # 识别手势
                    gesture, fingertip_coordinates = recognize_gesture(hand_landmarks)
                    
                    # 计算掌心中心
                    palm_center_x, palm_center_y = calculate_palm_center(hand_landmarks, image_width, image_height)
                    palm_depth = depth_image[palm_center_y, palm_center_x]
                    palm_3d = estimator.pixel_to_3d(palm_center_x, palm_center_y, palm_depth)

                    # 如果是“Pointing”手势，则计算指尖的3D坐标
                    if gesture == "Pointing":
                        finger_x, finger_y = fingertip_coordinates
                        finger_x = int(finger_x * image_width)
                        finger_y = int(finger_y * image_height)
                        finger_depth = depth_image[finger_y, finger_x]
                        finger_3d = estimator.pixel_to_3d(finger_x, finger_y, finger_depth)

                        # 打印指尖3D坐标
                        print(f"Pointing Finger Tip (3D): {finger_3d}")

                    # 打印掌心3D坐标
                    print(f"Palm Center (3D): {palm_3d}")
                    
                    # 可视化
                    cv2.circle(frame, (palm_center_x, palm_center_y), 10, (255, 0, 0), -1)
                    if gesture == "Pointing":
                        cv2.circle(frame, (finger_x, finger_y), 10, (0, 255, 0), -1)
                    
            cv2.imshow('Hand Gesture and 3D Coordinates', frame)
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

