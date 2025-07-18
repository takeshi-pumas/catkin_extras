#! /usr/bin/env python3
import os
import rospy, rospkg, math, yaml, sys, hsrb_interface
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose2D
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from robocup_hsr.srv import *
import robocup_hsr.msg
from tf import TransformListener
import tf, datetime
from tf.transformations import *
from rospkg import RosPack

# from boto3 import Session
# from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import subprocess
import json
from tempfile import gettempdir

import time
# import pyaudio
# import speech_recognition as sr

# from imutils import face_utils
import argparse
# import imutils
# import dlib
import cv2


from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# import gpt_super.gpt_super as gpt

from ultralytics import YOLO

from ros_openpose.msg import Frame

import ollama
import whisper
# from faster_whisper import WhisperModel

# import pyaudio
# import wave

import audio_stream.audio_record_listen as listen

import motion.action_robot as action_robot


import subprocess

from human_detector.srv import *

# Initialization -----------------------------------------------------------------------------------------

print("Taking control of the robot's interface")
rospy.init_node('rc25_hri')
robot = hsrb_interface.robot.Robot()



tts = robot.get('default_tts')
tts.language = tts.ENGLISH

# robot = hsrb_interface.robot.Robot()
global_tf = TransformListener()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
lidar = robot.get('base_scan')
gripper = robot.get('gripper')
# suction = robot.get('suction')
# omni_base.go_abs(0.7, 0.7, -2.3, 30.0)

ra = action_robot.RobotWithAction()

print('Joint controls online')

# frame_topic = rospy.get_param('~pub_topic')
frame_topic = "/frame"

# _CONNECTION_TIMEOUT = 10.0

rospy.sleep(5.) # can we try 5 seconds?

# print('Opening yaml parameters')
rospack = rospkg.RosPack()
# with open(rospack.get_path('robocup_hsr')+ '/include/study.yml', 'r') as file:
#     params = yaml.safe_load(file)
stream = open(rospack.get_path('robocup_hsr') + '/include/rc25_receptionist.yml', 'r')
params = yaml.load(stream)


lab_places = np.load(rospack.get_path('robocup_hsr') + '/scripts/behavior/data/rc23_receptionist.npy', allow_pickle=True).item()

lab_map = params['lab_map']


current_room = ''
### TO BE CHANGED
current_scan_room = 0
scan_rooms = [room for room in lab_places]
scan_rooms=['chair1', 'chair2','sofa1', 'sofa2','entrance', 'guest'] 


right_knee = 0
right_ankle = 0
left_knee = 0
left_ankle = 0

debug = True


_HAND_TF='hand_palm_link'

#Initialize mic------------------------------------------------------------------------------------------

# m = sr.Microphone()
# r = sr.Recognizer()

rp = RosPack()


#Helper functions -----------------------------------------------------------------------------------------

def callback_openpose(msg):
    global right_knee, right_ankle, left_knee, left_ankle
    # text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    # rospy.loginfo('%s\n' % text)


    for person in msg.persons: 
      right_knee = person.bodyParts[10]
      right_ankle = person.bodyParts[11]
      left_knee = person.bodyParts[13]
      left_ankle = person.bodyParts[14]
      neck_top = person.bodyParts[0]
      neck_top = person.bodyParts[1]

      return right_knee, right_ankle, left_knee, left_ankle

def finished():
  sys.exit(1)
def image_callback(data):
        img = self.cvbridge.imgmsg_to_cv2(data)

        img = cv2.addWeighted(img, self.brightness, img, 0,0)

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Filtering disabled - (Currently disable if you want to use segmentation.)
        result_img, detections = self.detect(img=img_rgb, filtering=False)

        # DEBUG
        result_img[self.img_center_y][self.img_center_x] = [0, 0, 255]


        # Publish img
        ros_img_msg = self.cvbridge.cv2_to_imgmsg(result_img)
        self.img_pub.publish(ros_img_msg)

        # Make json serializable as list instead of numpy array...
        detections_list = [detection.tolist() for detection in detections]

        detected_classes_dict = {}
        for detection in detections_list:
            class_key = int(detection[5])
            class_value = self.names[class_key]
            detected_classes_dict[class_key] = class_value



        detections_string = json.dumps(detections_list)
        names_string = json.dumps(self.names)
        detected_names_string = json.dumps(detected_classes_dict)


        self.box_pub.publish(detections_string)
        self.classes_pub.publish(names_string)
        self.detected_classes_pub.publish(detected_names_string)

def callback_yolo_pose(data):
  global left_shoulder, right_shoulder, left_ear, right_ear, nose
  img = self.cvbridge.imgmsg_to_cv2(data)
  img = cv2.addWeighted(img, self.brightness, img, 0,0)
  img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  model = YOLO("yolo11n-pose.pt")  # load an official model
  # model = YOLO("path/to/best.pt")  # load a custom model
  # 5 left_shoulder, 6 right_shoulder, 3 left_ear, 4 right_ear, 0 nose

  # Predict with the model
  results = model(img_rgb)  # predict on an image

  # Access the results
  for result in results:
      xy = result.keypoints.xy  # x and y coordinates
      xyn = result.keypoints.xyn  # normalized
      kpts = result.keypoints.data  # x, y, visibility (if available)
 

def go_to_position(move_goal):
  global table_pos, scan_num, current_room, room_pub
  # fill ROS message
  if debug:
    print("In go_to_position")
  pose = PoseStamped()
  pose.header.stamp = rospy.Time.now()
  pose.header.frame_id = "map"
  pose.pose.position = Point(move_goal.x, move_goal.y, move_goal.theta)
  
  if debug:
    print("location_set")
  try:
    if debug:
      print("moving base")
    omni_base.go_abs(move_goal.x, move_goal.y, move_goal.theta, 30.0)
  except:
    if debug:
      print("not moving")
    #go_to_position(move_goal)
  # room_pub.publish("start")

  if (omni_base.is_succeeded()):
    if debug:
      rospy.loginfo("Navigation Succeeded.")
    return True


def get_end_effector_pose(whole_body):

    t_pose = whole_body.get_current_pose()

    return t_pose   

def listen_question():
  result=""
  try:
    audio = listen.Recorder().record_once()
    model = whisper.load_model("base")
    result = model.transcribe(audio)
    print(result["text"])
    os.remove(audio)
  except:
    print("issue saving the audio file")

  # model_size = "base.en"

  # Run on GPU with FP16
  # model = WhisperModel(model_size, device="cuda", compute_type="float16")

  # or run on GPU with INT8
  # model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")
  # or run on CPU with INT8
  # model = WhisperModel(model_size, device="cpu", compute_type="int8")

  # segments, info = model.transcribe(audio, beam_size=5)

  # print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

  # result = ""

  # for segment in segments:
  #     result = result +  segment.text

  print(result)

  return result["text"]

def respond(content):
    global speed
    sudoPassword = 'neptune-canes'
    command = 'sudo systemctl start ollama.service'
    os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    os.popen("sudo -S %s"%(command), 'w').write('mypass')
    speak_this = ""

    messages = [
    {
        'role': 'user',
        'content': content,
    },
    ]
    if len(content.strip()) == 0:
        print('did not hear anything')
        # break

    for chunk in ollama.chat('initial-model', messages=messages, stream=True):
        # print(chunk['message']['content'], end='', flush=True)
        txt = chunk['message']['content']
        speak_this = speak_this + txt

    nwords = len(speak_this.split(" "))
    sperword = 0.32
    speed =  nwords*sperword

    os.system('sudo systemctl stop ollama.service')
    # command = 'sudo systemctl stop ollama.service'
    # os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    # os.popen("sudo -S %s"%(command), 'w').write('mypass')

    return speak_this

speed = 0
user_response = ""
audio_text = ""

def callback(data):
    global speed, user_response, audio_text

    data_str = data.data

    # print('Subscriber Received Data:', data_str)

    received_data = eval(data_str)
    # print('Received data:', received_data)
    # print('Received data type:', type(received_data))

    user_response = received_data['user_text']
    # print('user_text:', user_response)

    audio_text = received_data['audio_text']
    # print('audio_text:', audio_text)

    nwords = len(audio_text.split(" "))
    # print(nwords)
    sperword = 0.32
    speed =  nwords*sperword

    # t = datetime.datetime.strptime(received_data['audio_duration'], '%H:%M:%S.%f')
    # audio_duration = datetime.timedelta(hours = t.hour, minutes = t.minute, seconds = t.second, microseconds = t.microsecond)
    # print('audio_duration:', audio_duration)
    # print('audio duration type:', type(audio_duration))

    audio_offsets = received_data['audio_offsets']
    # print('audio_offsets:', audio_offsets)

    viseme_ids = received_data['viseme_ids']
    # print('viseme_ids:', viseme_ids)

    rospy_time = received_data['rospy_time']
    # print('rospy_time:', rospy_time)



def recieve_object():
  global right_knee, right_ankle, left_knee, left_ankle

  ra.move_to_neutral()
  # ra.move_arm(0.26349838338059968,
  #                   -0.7042003286985412,
  #                   -2.4245679361989403e-06,
  #                   -0.9406785587122649,
  #                   3.617331634231391e-07, True)


  gripper.command(1) #open gripper

  tts.say("Please place the object in my gripper")

  rospy.wait(5)
  #TODO: detect object in gripper

  tts.say("Please be careful, I will be closing my gripper")
  gripper.apply_force(1)

  #TODO: check if object grasped
  finger_dist = get_finger_distance()
  if finger_dist < 0.008: 
      success = False


def find_operator():
  global right_knee, right_ankle, left_knee, left_ankle, operator_location

  # go to center of the room
  # check if there is a person
  
  rospy.wait_for_service('detect_human')
  try:
    detect_human = rospy.ServiceProxy('detect_human', Human_detector) # ,Human_detectorResponse )
    resp = detect_human(x,y,z)

  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

  # if person found approach to about 50 com away
  # ask if they are oerator
  # if yes save robot location
  # else keep looking
  # repeate for other rooms until person found
 

def grasp_object():
  #TODO: Openpose - find arm

  #TODO: Recognize object being handed over
  return


def deliver_object(location):
  return


def return_to_operator():
  global operator_location
  ra.go_abs(operator_location)

      

# State Machine -----------------------------------------------------------------------------------------

def go_to_start_position():
  global photo_command_pub, current_room, current_pick_location, current_start_position, lab_places, current_scan_room, person_present, speed, user_response, audio_text, right_knee, right_ankle, left_knee, left_ankle
  # rospy.Subscriber(frame_topic, Frame, callback_openpose)
  # rospy.Subscriber("/robocanes/hsrb/head_rgbd_sensor/rgb/image_rectified", Image, callback_yolo_pose)  # check_open_chair_two_people("Julio")
  # rospy.Subscriber("/robocanes/hsrb/head_rgbd_sensor/depth_registered/image_rectified", Image, self.depth_callback)


  ra.move_to_go()

  # move to initial position
  # ra.go_abs(lab_map["start"][0], lab_map["start"][1], lab_map["start"][3])


  find_operator()





 

def begin():
  global current_room, command, lab_places, room_pub, photo_command_pub
  # rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, self.image_callback)
  # rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image", Image, human_detector)

  rospy.sleep(1)

  if debug:
    print(u'Let\'s meet some guests!')
    # room_pub = rospy.Publisher('/current_room', String, queue_size=10)
    print (lab_places)

  print("Ready, set, go!")
  return go_to_start_position


# Execution loop -------------------------------------------------------------------------------------------
## speak_wait("Adjusting for ambient noise, please be quiet.")

f = begin # starting state
# rate = rospy.Rate(10) # 10hz
# room_pub = rospy.Publisher('/current_room', String, queue_size=4)
while True:
  f = f()  # each function performs the transition logic and returns the next state