#! /usr/bin/env python3
import numpy as np
import os
import rospy, rospkg, math, yaml, sys, hsrb_interface
import actionlib
from actionlib_msgs.msg import GoalStatus

from ros_openpose.msg import Frame

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
frame_topic = "/frame"


def detect_keypoints(msg):
    global keypoints
    for person in msg.persons: 
        keypoints =  person.bodyParts

def recognize_action(keypoints):
    if keypoints is None: #or len(keypoints.shape) != 3:
        return "No person detected"
    
    left_wrist = keypoints[7].pixel  # Left wrist
    right_wrist = keypoints[4].pixel  # Right wrist
    left_elbow = keypoints[6].pixel  # Left elbow
    right_elbow = keypoints[3].pixel  # Right elbow
    left_shoulder = keypoints[5].pixel  # Left shoulder
    right_shoulder = keypoints[2].pixel  # Right shoulder

    actions = []
    
    if left_wrist.x != 0 or left_wrist.y!= 0 or left_elbow.x != 0 or left_elbow.y != 0 or left_shoulder.x != 0 or left_shoulder.y != 0:
        # Check for waving (left arm)
        # TODO: We need to look at waiving over time
        # if left_wrist.y < left_elbow.y and left_elbow.y < left_shoulder.y:
        #     actions.append("Waving")

        # Check for raising left arm
        if left_wrist.y < left_elbow.y and left_elbow.y < left_shoulder.y:
            actions.append("Raising left arm")
        
        # Check for pointing to the left
        if left_wrist.x < left_shoulder.x and abs(left_wrist.y - left_shoulder.y) < 50:
            actions.append("Pointing to the left")


    elif right_wrist.x != 0 or right_wrist.y!= 0 or right_elbow.x != 0 or right_elbow.y != 0 or right_shoulder.x != 0 or right_shoulder.y != 0:
        # Check for waving (right arm)
        # if right_wrist.y < right_elbow.y and right_elbow.y < right_shoulder.y:
        #     actions.append("Waving")

        # Check for raising right arm
        if right_wrist.y < right_elbow.y and right_elbow.y < right_shoulder.y:
            actions.append("Raising right arm")

        # Check for pointing to the right
        if right_wrist.x > right_shoulder.x and abs(right_wrist.y - right_shoulder.y) < 50:
            actions.append("Pointing to the right")


    if actions:
        return actions
    return 

def angle(shoulder, hip, knee):
    a = np.array([shoulder.x, shoulder.y])
    b = np.array([hip.x, hip.y])
    c = np.array([knee.x, knee.y])

    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    print(np.degrees(angle))
    return np.degrees(angle)

def recognize_posture(keypoints):
    if keypoints is None: #or len(keypoints.shape) != 3:
        return "No person detected"
    
    left_hip = keypoints[12].pixel  # Left hip
    right_hip = keypoints[9].pixel  # Right hip
    left_knee = keypoints[13].pixel  # Left knee
    right_knee = keypoints[10].pixel  # Right knee
    left_shoulder = keypoints[5].pixel  # Left shoulder
    right_shoulder = keypoints[2].pixel  # Right shoulder
    left_ankle = keypoints[14].pixel
    right_ankle = keypoints[11].pixel

    if left_hip.x == 0 or right_hip.x == 0 or left_knee.x == 0 or right_knee.x == 0 or left_shoulder.x == 0 or right_shoulder.x == 0:
        return
    elif left_hip.y == 0 or right_hip.y == 0 or left_knee.y == 0 or right_knee.y == 0 or left_shoulder.y == 0 or right_shoulder.y == 0:
        return

    actions = []

    posture_angle_left = angle(left_shoulder, left_hip, left_knee)
    posture_angle_right = angle(right_shoulder, right_hip, right_knee)


    if (posture_angle_left > 40 and posture_angle_right > 40) and (posture_angle_left < 110 and posture_angle_right < 110):
        return "Sitting person"

    if (posture_angle_left < 30 and posture_angle_right < 30) or (posture_angle_left > 130 and posture_angle_right > 130):

        # Check for vertical vs horizontal

        if((left_hip.y < left_knee.y) and (right_hip.y < right_knee.y)):
            return "Standing person"
        if abs(left_hip.y - right_hip.y) < 30 and abs(left_knee.y - right_knee.y) < 30:
            if abs(left_hip.y - left_knee.y) < 50:
                return "Lying person"


    return 

def main():
    global keypoints

    if not keypoints == None and not keypoints[11].pixel.x == 0.0: 

        actions = recognize_action(keypoints)
        postures = recognize_posture(keypoints)

        if actions:
            print(f"Detected actions: {actions}")
        if postures:
            print(f"Detected actions: {postures}")
    

if __name__ == "__main__":
    global keypoints
    rospy.Subscriber(frame_topic, Frame, detect_keypoints)
    keypoints = None

    while True:
        main()
        keypoints = None
