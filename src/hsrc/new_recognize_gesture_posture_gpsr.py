#! /usr/bin/env python3
import numpy as np
import os
import rospy, rospkg, math, yaml, sys, hsrb_interface
import actionlib
from actionlib_msgs.msg import GoalStatus
import cv2
import numpy as np
import rospy
from smach_utils2 import *
from human_detector.srv import Point_detector ,Point_detectorResponse 
from sensor_msgs.msg import Image , LaserScan , PointCloud2
pointing_detect_server = rospy.ServiceProxy('/detect_pointing' , Point_detector)


# Initialization -----------------------------------------------------------------------------------------
# 
# print("Taking control of the robot's interface")
rospy.init_node('rc24_gestures_postures')
robot = hsrb_interface.robot.Robot()



# tts = robot.get('default_tts')
# tts.language = tts.ENGLISH

# robot = hsrb_interface.robot.Robot()
# global_tf = TransformListener()
# omni_base = robot.get('omni_base')
# whole_body = robot.get('whole_body')
# lidar = robot.get('base_scan')
# gripper = robot.get('gripper')
# suction = robot.get('suction')
# omni_base.go_abs(0.7, 0.7, -2.3, 30.0)


# print('Joint controls online')

# frame_topic = rospy.get_param('~pub_topic')
#frame_topic = "/frame"

points_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2)
keypoints = get_keypoints(points_msg)

def recognize_action(keypoints):
    if keypoints is None: #or len(keypoints.shape) != 3:
        return "No person detected"
    
    left_wrist = keypoints[7, :2]  # Left wrist
    right_wrist = keypoints[4, :2]  # Right wrist
    left_elbow = keypoints[6, :2]  # Left elbow
    right_elbow = keypoints[3, :2]  # Right elbow
    left_shoulder = keypoints[5, :2]  # Left shoulder
    right_shoulder = keypoints[2, :2]  # Right shoulder

    actions = []
    
    if left_wrist[0] != 0 or left_wrist[1] != 0 or left_elbow[0] != 0 or left_elbow[1] != 0 or left_shoulder[0] != 0 or left_shoulder[1] != 0:
        # Check for waving (left arm)
        # TODO: We need to look at waiving over time
        # if left_wrist[1] < left_elbow[1] and left_elbow[1] < left_shoulder[1]:
        #     actions.append("Waving")

        # Check for raising left arm
        if left_wrist[1] < left_elbow[1] and left_elbow[1] < left_shoulder[1]:
            actions.append("Raising left arm")
        
        # Check for pointing to the left
        if left_wrist[0] < left_shoulder[0] and abs(left_wrist[1] - left_shoulder[1]) < 50:
            actions.append("Pointing to the left")


    elif right_wrist[0] != 0 or right_wrist[1]!= 0 or right_elbow[0] != 0 or right_elbow[1] != 0 or right_shoulder[0] != 0 or right_shoulder[1] != 0:
        # Check for waving (right arm)
        # if right_wrist[1] < right_elbow[1] and right_elbow[1] < right_shoulder[1]:
        #     actions.append("Waving")

        # Check for raising right arm
        if right_wrist[1] < right_elbow[1] and right_elbow[1] < right_shoulder[1]:
            actions.append("Raising right arm")

        # Check for pointing to the right
        if right_wrist[0] > right_shoulder[0] and abs(right_wrist[1] - right_shoulder[1]) < 50:
            actions.append("Pointing to the right")


    if actions:
        return actions
    return 

def angle(shoulder, hip, knee):
    a = np.array([shoulder[0], shoulder[1]])
    b = np.array([hip[0], hip[1]])
    c = np.array([knee[0], knee[1]])

    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    print(np.degrees(angle))
    return np.degrees(angle)

def recognize_posture(keypoints):
    if keypoints is None: #or len(keypoints.shape) != 3:
        return "No person detected"
    
    left_hip = keypoints[12, :2]  # Left hip
    right_hip = keypoints[9, :2]  # Right hip
    left_knee = keypoints[13, :2]  # Left knee
    right_knee = keypoints[10, :2]  # Right knee
    left_shoulder = keypoints[5, :2]  # Left shoulder
    right_shoulder = keypoints[2, :2]  # Right shoulder
    left_ankle = keypoints[14, :2]
    right_ankle = keypoints[11, :2]

    if left_hip[0] == 0 or right_hip[0] == 0 or left_knee[0] == 0 or right_knee[0] == 0 or left_shoulder[0] == 0 or right_shoulder[0] == 0:
        return
    elif left_hip[1] == 0 or right_hip[1] == 0 or left_knee[1] == 0 or right_knee[1] == 0 or left_shoulder[1] == 0 or right_shoulder[1] == 0:
        return

    actions = []

    posture_angle_left = angle(left_shoulder, left_hip, left_knee)
    posture_angle_right = angle(right_shoulder, right_hip, right_knee)


    if (posture_angle_left > 40 and posture_angle_right > 40) and (posture_angle_left < 110 and posture_angle_right < 110):
        return "Sitting person"

    if (posture_angle_left < 30 and posture_angle_right < 30) or (posture_angle_left > 130 and posture_angle_right > 130):

        # Check for vertical vs horizontal

        if((left_hip[1] < left_knee[1]) and (right_hip[1] < right_knee[1])):
            return "Standing person"
        if abs(left_hip[1] - right_hip[1]) < 30 and abs(left_knee[1] - right_knee[1]) < 30:
            if abs(left_hip[1] - left_knee[1]) < 50:
                return "Lying person"


    return 

def main():
    global keypoints

    if not keypoints == None and not keypoints[11, :][0] == 0.0: 

        actions = recognize_action(keypoints)
        postures = recognize_posture(keypoints)

        if actions:
            print(f"Detected actions: {actions}")
        if postures:
            print(f"Detected actions: {postures}")
    

if __name__ == "__main__":
    # global keypoints
    # rospy.Subscriber(frame_topic, Frame, detect_keypoints)
    # keypoints = None

    while True:
        main()
        keypoints = None
