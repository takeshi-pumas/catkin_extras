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

from ros_openpose.msg import Frame

import ollama
import whisper
# from faster_whisper import WhisperModel

# import pyaudio
# import wave

import audio_stream.audio_record_listen as listen

import motion.action_robot as action_robot


import subprocess
# Initialization -----------------------------------------------------------------------------------------

print("Taking control of the robot's interface")
rospy.init_node('rc23_receptionist')
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
stream = open(rospack.get_path('robocup_hsr') + '/include/rc24_receptionist.yml', 'r')
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


      return right_knee, right_ankle, left_knee, left_ankle

def finished():
  sys.exit(1)
 
def get_head_point(search_loc):
  global head_enabled
  head_curr_alfa = whole_body.joint_state.position[9]
  head_curr_beta = whole_body.joint_state.position[10]
  search_pos = PoseStamped()
  search_pos.header.frame_id = 'map'
  search_pos.header.stamp = rospy.Time(0)
  search_pos.pose.position.x = search_loc[0]
  search_pos.pose.position.y = search_loc[1]
  search_pos.pose.position.z = search_loc[2]
  if debug:
    print("GAZE POSE X:", search_loc[0])
    print("GAZE POSE Y:", search_loc[1])
    print("Z: ", search_loc[2])

  target_2_head = PoseStamped()
  global_tf.waitForTransform('head_rgbd_sensor_link', 'map', rospy.Time(0), rospy.Duration(1.0))
  target_2_head = global_tf.transformPose('head_rgbd_sensor_link', search_pos)
  transform = global_tf.lookupTransform('head_rgbd_sensor_link', 'map', rospy.Time(0))
  pos = transform[0]
  a = target_2_head.pose.position.x
  b = target_2_head.pose.position.z
  c = target_2_head.pose.position.y

  alfa = np.arctan(a/b)
  beta = np.arctan(c/b)
  if (a <0  and b < 0) or (b < 0 and a >0):
    alfa = alfa + np.pi
  if (c <0  and b < 0) or (b < 0 and c >0):
    beta = beta + np.pi

  target_alfa = head_curr_alfa - alfa
  target_beta = head_curr_beta - beta

  if(target_beta > 2.35):
    target_beta = np.pi - target_beta
    target_alfa = target_alfa + np.pi

  if (target_alfa < - 4.54 ):
    target_alfa = 2*np.pi + target_alfa
  elif (target_alfa > 2.445):
    target_alfa = -(2*np.pi  - target_alfa)

  if debug:
    print ("current tilt joint = " + str(head_curr_beta)) 
    print ("current alfa = " + str(head_curr_alfa)) 
    print ("beta = " + str(-beta)) 
    print ("alfa = " + str(-alfa)) 
    print ("target_beta = " + str(target_beta)) 
    print ("target_alfa = " + str(target_alfa)) 

  whole_body.move_to_joint_positions({'head_pan_joint': np.clip(target_alfa,-3.689,1.595), 'head_tilt_joint': np.clip(target_beta,-1.1, 0.45)})



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


def faces():
  global all_objects, pick_location, place_location, pick_object, lab_places, all_object_names, all_object_locations, current_room, num_picked_objects, current_scan_room

  if len(all_object_locations[scan_rooms[current_scan_room]]) == 0:
    # print(u'no objects left to pick')
    return list_objects

  new_pose = PoseStamped()
  global_tf.waitForTransform('base_footprint', all_object_locations[scan_rooms[current_scan_room]][0].header.frame_id, all_object_locations[scan_rooms[current_scan_room]][0].header.stamp, rospy.Duration(4.0))
  new_pose = global_tf.transformPose('base_footprint', all_object_locations[scan_rooms[current_scan_room]][0])
  min_dist =  new_pose.pose.position.x**2 + new_pose.pose.position.y**2
  min = 0
  for i in range(len(all_object_locations[scan_rooms[current_scan_room]])):
    global_tf.waitForTransform('base_footprint', all_object_locations[scan_rooms[current_scan_room]][i].header.frame_id, all_object_locations[scan_rooms[current_scan_room]][i].header.stamp, rospy.Duration(4.0))
    new_pose = global_tf.transformPose('base_footprint', all_object_locations[scan_rooms[current_scan_room]][i])
    mark_dist_i =  new_pose.pose.position.x**2 + new_pose.pose.position.y**2
    if mark_dist_i < min_dist:
      min_dist = mark_dist_i
      min = i
  pick_object = all_object_names[scan_rooms[current_scan_room]][min]
  pick_pose = all_object_locations[scan_rooms[current_scan_room]][min]
  marker = Marker()
  marker.pose = pick_pose.pose
  marker.ns = pick_object[0]

  get_head_point(marker.pose.position) #should be x,y,z


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



def name_choice(name):
    if "sophie" in name.lower():
        return "Sophie"
    elif "julia" in name.lower():
        return "Julia"
    elif "emma" in name.lower():
        return "Emma"
    elif "sara" in name.lower():
        return "Sara"
    elif "laura" in name.lower():
        return "Laura"
    elif "hayley" in name.lower():
        return "Hayley"
    elif "susan" in name.lower():
        return "Susan"
    elif "fleur" in name.lower():
        return "Fleur"
    elif "gabrielle" in name.lower():
        return "Gabriëlle"
    elif "robin" in name.lower():
        return "Robin"
    elif "john" in name.lower():
        return "John"
    elif "liam" in name.lower():
        return "Liam"
    elif "lucas" in name.lower():
        return "Lucas"
    elif "william" in name.lower():
        return "William"
    elif "kevin" in name.lower():
        return "Kevin"
    elif "jesse" in name.lower():
        return "Jesse"
    elif "noah" in name.lower():
        return "Noah"
    elif "harrie" in name.lower():
        return "Harrie"
    elif "peter" in name.lower():
        return "Peter"
    else:
        return "Name not found"

def drink_choice(drink):
    if "coke" in drink.lower():
        return "Coke"
    elif "fanta" in drink.lower():
        return "Fanta"
    elif "ice tea" in drink.lower():
        return "Ice Tea"
    elif "dubbelfris" in drink.lower():
        return "Dubbelfris"
    elif "water" in drink.lower():
        return "Water"
    elif "milk" in drink.lower():
        return "Milk"
    elif "big coke" in drink.lower():
        return "Big Coke"
    else:
        return "Drink not found"


def check_open_chair():
  global right_knee, right_ankle, left_knee, left_ankle
  chair_1 = 0
  chair_2 = 0
  sofa_1 = 0
  sofa_2 = 0
    # whole_body.move_to_joint_positions({'head_tilt_joint': -0.37, 'head_pan_joint': 1.1})
  ra.move_head(0.8, -0.75)
  rospy.sleep(1.)
  right_knee = 0
  right_ankle = 0
  left_knee = 0
  left_ankle = 0
  rospy.sleep(3.)
  if(not left_ankle == None and not left_ankle == 0 and  (left_ankle.pixel.x > 0 and left_ankle.pixel.x > 160 or left_ankle.pixel.x < 480 or right_ankle.pixel.x > 160 or right_ankle.pixel.x < 480)):
    print("found host")
    chair_1 = 1
  else:
    ra.move_head(-0.2, -0.75)
    rospy.sleep(4.)
    if(not left_ankle == None and not left_ankle == 0 and  left_ankle.pixel.x > 0 and (left_ankle.pixel.x < 320 or right_ankle.pixel.x < 320)):
      print("found host")
      sofa_1 = 1
    else:
      if(not left_ankle == None and not left_ankle == 0 and  left_ankle.pixel.x > 0 and (left_ankle.pixel.x > 320 or right_ankle.pixel.x > 320)):
        print("found host")
        sofa_2 = 1
      else:
        ra.move_head(-1.0, -0.75)
        rospy.sleep(4.)
        if(not left_ankle == None and not left_ankle == 0 and  left_ankle.pixel.x > 0 and (left_ankle.pixel.x > 160 or left_ankle.pixel.x < 480 or right_ankle.pixel.x > 160 or right_ankle.pixel.x < 480)):
          print("found host")
          chair_2 = 1
  return chair_1, chair_2,sofa_1, sofa_2

def check_open_chair_two_people(name_1, drink_1):
  global right_knee, right_ankle, left_knee, left_ankle

  chair_1 = 0
  chair_2 = 0
  sofa_1 = 0
  sofa_2 = 0
  ra.move_head(0.8, -0.75)
  rospy.sleep(1.)
  right_knee = 0
  right_ankle = 0
  left_knee = 0
  left_ankle = 0
  rospy.sleep(3.)
  if(not left_ankle == None and not left_ankle == 0 and  left_ankle.pixel.x > 0 and (left_ankle.pixel.x > 160 or left_ankle.pixel.x < 480 or right_ankle.pixel.x > 160 or right_ankle.pixel.x < 480)):
    print("found host")
    chair_1 = 1
    ra.move_head(1.1, 0.0)
    rospy.sleep(2.)
    tts.say("Smile.")
    photo_command_pub.publish("Go chair_1") 
    # rospy.sleep(10)
    while not os.path.exists('/home/robocanes/hsr_robocanes/chair_1_person_description.txt'):
      x = 1
    with open("/home/robocanes/hsr_robocanes/chair_1_person_description.txt", "r") as file:
      description = file.read()
      if("Yes" in description):
        tts.say("Here is our first guest " + name_1)
        rospy.sleep(3.)
        tts.say("Their favourite drink is " + drink_1)

        chair_1 = 2
      else:
        tts.say("Here is our host, John. His favourite drink is milk")
        rospy.sleep(3.)

    ra.move_head(1.3, -0.5)

  ra.move_head(-0.2, -0.75)
  rospy.sleep(4.)
  if(not left_ankle == None and not left_ankle == 0 and  left_ankle.pixel.x > 0 and (left_ankle.pixel.x < 320 or right_ankle.pixel.x < 320)):
    print("found person sofa_1")
    sofa_1 = 1
    ra.move_head(0.1, 0.0)
    rospy.sleep(2.)
    tts.say("Smile.")
    photo_command_pub.publish("Go sofa_1") 
    # rospy.sleep(10)
    while not os.path.exists('/home/robocanes/hsr_robocanes/sofa_1_person_description.txt'):
      x = 1
    with open("/home/robocanes/hsr_robocanes/sofa_1_person_description.txt", "r") as file:
      description = file.read()
      if("Yes" in description):
        tts.say("Here is our first guest " + name_1)
        rospy.sleep(3.)
        tts.say("Their favourite drink is " + drink_1)
        sofa_1 = 2
      else:
        tts.say("Here is our host, John. His favourite drink is milk")
        rospy.sleep(3.)
  
  ra.move_head(-0.2, -0.75)
  rospy.sleep(1.)
  right_knee = 0
  right_ankle = 0
  left_knee = 0
  left_ankle = 0
  rospy.sleep(3.)
  if(not left_ankle == None and not left_ankle == 0 and  left_ankle.pixel.x > 0 and (left_ankle.pixel.x > 320 or right_ankle.pixel.x > 320)):
    print("found person sofa_2")
    sofa_2 = 1
    ra.move_head(-0.2, 0.0)
    rospy.sleep(2.)
    tts.say("Smile.")
    photo_command_pub.publish("Go sofa_2") 
    # rospy.sleep(10)
    while not os.path.exists('/home/robocanes/hsr_robocanes/sofa_2_person_description.txt'):
      x = 1
    with open("/home/robocanes/hsr_robocanes/sofa_2_person_description.txt", "r") as file:
      description = file.read()
      if("Yes" in description):
        tts.say("Here is our first guest " + name_1)
        rospy.sleep(3.)
        tts.say("Their favourite drink is " + drink_1)
        sofa_2 = 2
      else:
        tts.say("Here is our host, John. His favourite drink is milk")
    ra.move_head(0.0, -0.3)

  ra.move_head(-1.0, -0.75)
  rospy.sleep(4.)
  if(not left_ankle == None and not left_ankle == 0 and  left_ankle.pixel.x > 0 and (left_ankle.pixel.x > 160 or left_ankle.pixel.x < 480 or right_ankle.pixel.x > 160 or right_ankle.pixel.x < 480)):
    print("found person chair_2")
    chair_2 = 1
    ra.move_head(-1.0, 0.0)
    rospy.sleep(2.)
    tts.say("Smile.")
    photo_command_pub.publish("Go chair_2") 
    # rospy.sleep(10)
    while not os.path.exists('/home/robocanes/hsr_robocanes/chair_2_person_description.txt'):
      x = 1
    with open("/home/robocanes/hsr_robocanes/chair_2_person_description.txt", "r") as file:
      description = file.read()
      if("Yes" in description):
        tts.say("Here is our first guest " + name_1)
        rospy.sleep(3.)
        tts.say("Their favourite drink is " + drink_1)
        chair_2 = 2
      else:
        tts.say("Here is our host, John. His favourite drink is milk")
        rospy.sleep(3.)
    ra.move_head(-1.0, -0.3)

  return chair_1, chair_2,sofa_1, sofa_2


      

# State Machine -----------------------------------------------------------------------------------------

def go_to_start_position():
  global photo_command_pub, current_room, current_pick_location, current_start_position, lab_places, current_scan_room, person_present, speed, user_response, audio_text, right_knee, right_ankle, left_knee, left_ankle
  rospy.Subscriber(frame_topic, Frame, callback_openpose)
  # check_open_chair_two_people("Julio")
#   response_text = listen_question()


# def stop():
  ra.move_to_go()

  ra.go_abs(lab_map["start"][0], lab_map["start"][1], lab_map["start"][3])

  # Move to photo position
  ra.go_abs(lab_map["photo_position"][0], lab_map["photo_position"][1], lab_map["photo_position"][3])

  # TODO: see whole person - check using openpose that we have a skeleton or using vision

  # Send message to take photo
  whole_body.move_to_joint_positions({'head_tilt_joint': 0.15})
  rospy.sleep(1.)
  tts.say("Smile so I can take your picture to remember you.")
  rospy.sleep(2.)
  photo_command_pub.publish("Go guest_1") # we do this only for the 1st person to describe them to the 2nd
  rospy.sleep(3.)

  # Go to greet guest
  ra.go_abs(lab_map["door"][0], lab_map["door"][1], lab_map["door"][3])  
  # rospy.sleep(6.)
  # tts.say("Hello")
  # rospy.sleep(1)
  # text = listen_question()
  # response_text = respond(text)
  # tts.say(response_text)
  # rospy.Subscriber("gpt_super", String, callback)
  # response_text = respond("Introduce yourself. Ask the arriving guest for their name.")
  # whole_body.move_to_joint_positions({'head_tilt_joint': 0.3})
  whole_body.move_to_joint_positions({'arm_lift_joint':0.45,
                                      'arm_flex_joint': -1.9,
                                      'head_tilt_joint': 0.1})
  tts.say("Hi, my name is Palpy. What is your name? Please talk close to the mic on my head.")
  rospy.sleep(4.2)
  response_text = ""
  try:
    response_text = listen_question()
  except:
    tts.say("Sorry, could you repeat?")
    rospy.sleep(1.5)
    response_text = listen_question()
  # print(user_response)

  name_1 = name_choice(response_text.lower())
  print("The name is " + name_1)

  # name_1 = "Tom"

  tts.say("And what is your favourite drink?")
  rospy.sleep(2.)
  response_text = ""
  try:
    response_text = listen_question()
  except:
    tts.say("Sorry, could you repeat?")
    rospy.sleep(1.5)
    response_text = listen_question()
  # print(user_response)

  drink_1 = drink_choice(response_text.lower())
  print("The drink is " + drink_1)

  ra.move_to_go()
  tts.say("Please, follow me.")

  # Go to the sitting
  ra.go_abs(lab_map["host"][0], lab_map["host"][1], lab_map["host"][3])

  tts.say("Please, stand to my left")
  ra.move_head(1.2, 0.5)

  rospy.sleep(4.)

  tts.say("Looking for the host")

  chair_1, chair_2,sofa_1, sofa_2 = check_open_chair()

  whole_body.move_to_joint_positions({'head_tilt_joint': 0.0})

  tts.say("This is John, our host. His favourite drink is milk")
  rospy.sleep(2.5)
  # TODO: look for guest
  ra.move_head(1.2, 0.5)

  tts.say("John, this is " + name_1)
  rospy.sleep(2.6)
  tts.say("Their favorite drink in " + drink_1)
  rospy.sleep(3)

  whole_body.move_to_go()

  if(chair_1 == 0 ):
    tts.say("left chair looks open")
    rospy.sleep(1.2)
  elif(chair_2 == 0 ):
    tts.say("right chair looks open")
    rospy.sleep(1.2)
  elif(sofa_1 == 0 ):
    tts.say("left side of the sofa looks open")
    rospy.sleep(1.6)
  elif(sofa_2 == 0 ):
    tts.say("right side of the sofa looks open")
    rospy.sleep(1.6)
  
  

  tts.say("Please, make yourself comfortable and sit down.")
  
  rospy.sleep(5.)


  # #--------------- Second guest ------------------------------


  # Go to greet guest
  ra.go_abs(lab_map["door"][0], lab_map["door"][1], lab_map["door"][3])  

  # rospy.sleep(6.)
  # tts.say("Hello")
  # rospy.sleep(1)
  # text = listen_question()
  # response_text = respond(text)
  # tts.say(response_text)
  # rospy.Subscriber("gpt_super", String, callback)
  # response_text = respond("Introduce yourself. Ask the arriving guest for their name.")
  # tts.say(response_text)
  # rospy.sleep(speed + 2.)
  whole_body.move_to_joint_positions({'arm_lift_joint':0.45,
                                      'arm_flex_joint': -1.9,
                                      'head_tilt_joint': 0.1})
  tts.say("Hi, my name is Palpy. What is your name? Please talk close to the mic on my head.")
  rospy.sleep(4.2)
  response_text = ""
  try:
    response_text = listen_question()
  except:
    tts.say("Sorry, could you repeate?")
    rospy.sleep(1.)
    response_text = listen_question()
  print(user_response)

  name = name_choice(response_text.lower())
  print("The name is " + name)

  tts.say("Nice to meet you. What is your favourite drink?")
  rospy.sleep( 2.5)
  
  response_text = ""
  try:
    response_text = listen_question()
  except:
    tts.say("Sorry, could you repeate?")
    rospy.sleep(1.)
    response_text = listen_question()
  # print(user_response)

  drink = drink_choice(response_text.lower())
  print("The drink is " + drink)

  ra.move_to_go()
  tts.say("Please, follow me.")

  # Go to the sitting
  ra.go_abs(lab_map["host"][0], lab_map["host"][1], lab_map["host"][3])

  tts.say("Please, stand to my left")
  ra.move_head(1.0, 0.3)

  rospy.sleep(4.)


  # whole_body.move_to_joint_positions({'head_tilt_joint': -0.1, 'head_pan_joint': 0.5})
  ra.move_head(0.5, -0.1)

  # # check if you see a spot or human
  # name = "Morgan"
  # drink = "cola"

  tts.say("Looking for the host")

  chair_1, chair_2,sofa_1, sofa_2 = check_open_chair_two_people(name_1, drink_1)

  # chair_1=2 
  # chair_2 =1 
  # sofa_1 = 0 
  # sofa_2 = 0


  whole_body.move_to_joint_positions({'head_tilt_joint': 0.0})

  # tts.say("This is John, our host. ")
  # rospy.sleep(2.5)
  # tts.say("And this is our earlier guest, " + name_1)
  # rospy.sleep(3.)
  # TODO: look for guest
  ra.move_head(1.0, 0.3)
  tts.say("Everyone, this is " + name)
  rospy.sleep(2.6)
  tts.say("Their favourite drink is " + drink)
  rospy.sleep(3.)

  #Look at first guest

  if(chair_1 == 2 ):
    ra.move_head(1.3, 0.0)
  elif(chair_2 == 2 ):
    ra.move_head(-1.0, 0.0)
  elif(sofa_1 == 2 ):
    ra.move_head(0.3, 0.0)
  elif(sofa_2 == 2 ):
    ra.move_head(-0.3, 0.0)

  # TODO: Describe first guest
  with open("/home/robocanes/hsr_robocanes/guest_1_person_description.txt", "r") as file:
    description = file.read()

    # tts.say(description)
    promp = "Describe " + name_1 + " in your own words. " + description
    gen = respond(promp)
    tts.say(gen)
    rospy.sleep(speed)

  # TODO: look for guest
  ra.move_head(1.0, 0.3)

  whole_body.move_to_go()

  if(chair_1 == 0 ):
    tts.say("left chair looks open")
    rospy.sleep(1.2)
  elif(chair_2 == 0 ):
    tts.say("right chair looks open")
    rospy.sleep(1.2)
  elif(sofa_1 == 0 ):
    tts.say("left side of the sofa looks open")
    rospy.sleep(1.6)
  elif(sofa_2 == 0 ):
    tts.say("right side of the sofa looks open")
    rospy.sleep(1.6)
  
  tts.say("Please, make yourself comfortable and sit down.")
  
  rospy.sleep(3.)

 

def begin():
  global current_room, command, lab_places, room_pub, photo_command_pub
  photo_command_pub = rospy.Publisher('/take_photo', String, queue_size=10)




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