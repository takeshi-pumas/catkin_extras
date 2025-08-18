#! /usr/bin/env python3

# General imports.
import math
import os
# import gpt_super.gpt_super as gpt
import json
import numpy as np
import re
import string
import sys
import threading
import yaml
# import speech_recognition as sr
from xml.dom import minidom
import json

# ROS imports.
import rospy
from hsrb_interface import Robot
from rospkg import RosPack; rp = RosPack()
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# Custom imports.
from robocup_hsr.srv import *
import robocup_hsr.msg

from transformers import VisionEncoderDecoderModel, ViTImageProcessor, AutoTokenizer, pipeline, AutoProcessor, AutoImageProcessor, LlavaForConditionalGeneration
import torch
import PIL.Image
from ros_openpose.msg import Frame

import cv2

from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose2D

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import ollama, whisper
import audio_stream.audio_record_listen as audion_listen

import motion.action_robot as action_robot

# Initializing perceptor for picking
# rospy.wait_for_service("toggle_perceptor")
# _toggle_perceptor = rospy.ServiceProxy("toggle_perceptor", toggle_map_object_perceptor)
# _toggle_perceptor()
model = whisper.load_model("base")
cvbridge = CvBridge()
photo_command_pub = rospy.Publisher('/take_photo', String, queue_size=10)

class rc23_gpsr:
    def establish_config(self):
        self.config_file_name = 'rc24_gpsr_eindhoven_md.yaml'

        self.config_path = os.path.join(self.repo_path, 'config')
        self.config_file_path = os.path.join(self.config_path, self.config_file_name)
        print(self.config_file_path)

        # DEBUG
        # print('config file path:', self.config_file_path)

        '''Read designated yaml config file.'''
        with open(self.config_file_path, 'r') as stream:
            try:
                self.rc24_gpsr_yaml = yaml.safe_load(stream)

                self.robot_dict = self.rc24_gpsr_yaml['robot']
                self.grammar_dict = self.rc24_gpsr_yaml['grammar']
                self.locations_pos_dict = self.rc24_gpsr_yaml['locations_pos']
                self.place_loc = self.rc24_gpsr_yaml['place_loc']

                # DEBUG
                print('robot_dict:', self.robot_dict)
                print('grammar_dict:', self.grammar_dict)
                print('locations_pos_dict:', self.locations_pos_dict)
            except Exception as e:
                print('rc23_gpsr establish_config yaml read exception:', e)

        # Robot
        self.real = self.robot_dict['real']
        self.instruct_pos = eval(self.robot_dict['instruct_pos'])
        self.move = False

        # Grammar
        self.verbs_dict = self.grammar_dict['verbs']

        self.take_verbs = self.verbs_dict['take']
        self.place_verbs = self.verbs_dict['place']
        self.deliver_verbs = self.verbs_dict['deliver']
        self.bring_verbs = self.verbs_dict['bring']
        self.go_verbs = self.verbs_dict['go']
        self.find_verbs = self.verbs_dict['find']
        self.talk_verbs = self.verbs_dict['talk']
        self.answer_verbs = self.verbs_dict['answer']
        self.meet_verbs = self.verbs_dict['meet']
        self.tell_verbs = self.verbs_dict['tell']
        self.greet_verbs = self.verbs_dict['greet']
        self.remember_verbs = self.verbs_dict['remember']
        self.count_verbs = self.verbs_dict['count']
        self.describe_verbs = self.verbs_dict['describe']
        self.offer_verbs = self.verbs_dict['offer']
        self.follow_verbs = self.verbs_dict['follow']
        self.guide_verbs = self.verbs_dict['guide']
        self.accompany_verbs = self.verbs_dict['accompany']

        # Locations Poses
        self.locations_pos = {}

        print(f'SELF.LOCATIONS_ACTION in config: {self.locations_action}')

        # Hallway
        if 'hallway' in self.locations_action:
            self.hallway_room = self.locations_pos_dict['hallway']
            self.hallway_names = list(self.hallway_room.keys())
            self.locations_pos.update(self.hallway_room)
        
        # Office
        if 'office' in self.locations_action:
            self.office_room = self.locations_pos_dict['office']
            self.office_names = list(self.office_room.keys())
            self.locations_pos.update(self.office_room)

        # Kitchen
        if 'kitchen' in self.locations_action:
            self.kitchen_room = self.locations_pos_dict['kitchen']
            self.kitchen_names = list(self.kitchen_room.keys())
            self.locations_pos.update(self.kitchen_room)

        # Living Room
        if 'living room' in self.locations_action:
            self.living_room_room = self.locations_pos_dict['living room']
            self.living_room_names = list(self.living_room_room.keys())
            self.locations_pos.update(self.living_room_room)

        # Corridor
        # if 'lab' in self.locations_action:
        #     self.corridor_room = self.locations_pos_dict['lab']
        #     self.corridor_names = list(self.corridor_room.keys())
        #     self.locations_pos.update(self.corridor_room)

        # # Corridor
        # if 'corridor' in self.locations_action:
        #     self.corridor_room = self.locations_pos_dict['corridor']
        #     self.corridor_names = list(self.corridor_room.keys())
        #     self.locations_pos.update(self.corridor_room)

        # # Bedroom
        # if 'bedroom' in self.locations_action:
        #     self.bedroom_room = self.locations_pos_dict['bedroom']
        #     self.bedroom_names = list(self.bedroom_room.keys())
        #     self.locations_pos.update(self.bedroom_room)

        # # Study
        # if 'study' in self.locations_action:
        #     self.study_room = self.locations_pos_dict['study']
        #     self.study_names = list(self.study_room.keys())
        #     self.locations_pos.update(self.study_room)

        # # Dining room
        # if 'dining room' in self.locations_action:
        #     self.dining_room_room = self.locations_pos_dict['dining room']
        #     self.dining_room_names = list(self.dining_room_room.keys())
        #     self.locations_pos.update(self.dining_room_room)

        # # Living room
        # if 'living room' in self.locations_action:
        #     self.living_room_room = self.locations_pos_dict['living room']
        #     self.living_room_names = list(self.living_room_room.keys())
        #     self.locations_pos.update(self.living_room_room)

        # # Kitchen
        # if 'kitchen' in self.locations_action:
        #     self.kitchen_room_room = self.locations_pos_dict['kitchen']
        #     self.kitchen_room_names = list(self.kitchen_room_room.keys())
        #     self.locations_pos.update(self.kitchen_room_room)

        self.locations_names = list(self.locations_pos.keys())

        # DEBUG
        print('locations_pos:', self.locations_pos)
        print('locations_names:', self.locations_names)

    def read_elements(self, type, file_path):
        elements = []
        file = minidom.parse(file_path)

        # DEBUG
        # print('Parse xml file:', file)

        if type == 'gestures':
            # DEBUG
            print('reading xml gestures...')

            tag_elements = file.getElementsByTagName('gesture')

            for element in tag_elements: 
                name = element.attributes['name'].value.lower()
                difficulty = element.attributes['difficulty'].value.lower()
                elements.append({'name': name, 'difficulty': difficulty})

        elif type == 'rooms':
            # DEBUG
            print('reading xml rooms...')

            tag_elements = file.getElementsByTagName('room')

            for element in tag_elements:
                name = element.attributes['name'].value.lower()
                elements.append({'name': name})

        elif type == 'locations':
            # DEBUG
            print('reading xml locations...')

            tag_elements = file.getElementsByTagName('location')

            for element in tag_elements:
                name = element.attributes['name'].value.lower()
                elements.append({'name': name})

        elif type == 'names':
            # DEBUG
            print('reading xml names...')

            tag_elements = file.getElementsByTagName('name')

            for element in tag_elements:
                name = element.firstChild.data.lower()
                elements.append({'name': name})

        elif type == 'objects_categories':
            # DEBUG
            print('reading xml object categories...')

            categories = file.getElementsByTagName('category')

            for category in categories:
                # DEBUG
                # print('category:', category)

                category_name = category.attributes['name'].value.lower()

                default_location = category.attributes['defaultLocation'].value.lower()

                room = category.attributes['room'].value.lower()

                category_list = []

                # DEBUG
                # print('category attributes:', category.attributes)

                # DEBUG
                # print('object categories name :', category_name)

                for item in category.childNodes:
                    # DEBUG
                    # print('item:', item)
                    # print('item node type:', item.nodeType)

                    if item.nodeType == item.ELEMENT_NODE:

                        item_name = item.getAttribute('name')

                        # DEBUG
                        # print('object category item:', item_name)

                        category_list.append(item_name)

                elements.append({'name': category_name, 'defaultLocation': default_location, 'room': room, 'categoryList': category_list})
                
        elif type == 'objects':
             # DEBUG
            print('reading xml objects...')

            tag_elements = file.getElementsByTagName('object')

            for element in tag_elements:
                name = element.attributes['name'].value.lower()
                type = element.attributes['type'].value.lower()
                difficulty = element.attributes['difficulty'].value.lower()
                elements.append({'name': name, 'type': type, 'difficulty': difficulty})
        elif type == 'questions':
            # DEBUG
            print('reading xml questions...')

            q_elements = file.getElementsByTagName('q')
            a_elements = file.getElementsByTagName('a')

            for q, a in zip(q_elements, a_elements):
                q = q.firstChild.data.lower()
                a = a.firstChild.data.lower()
                elements.append({'q': q, 'a': a})
        else:
            # DEBUG
            print('unknown xml type...')

        return elements
    
    def read_data(self, file_path):
        with open(file_path, 'r') as file:
            data = file.read()
        return data

    def parse_names(self, data):
        parsed_names = re.findall(r'\|\s*([A-Za-z]+)\s*\|', data, re.DOTALL)
        parsed_names = [name.strip() for name in parsed_names]

        if parsed_names:
            return parsed_names[1:]
        else:
            warnings.warn("List of names is empty. Check content of names markdown file")
            return []

    def parse_locations(self, data):
        parsed_locations = re.findall(r'\|\s*([0-9]+)\s*\|\s*([A-Za-z,\s, \(,\)]+)\|', data, re.DOTALL)
        parsed_locations = [b for (a, b) in parsed_locations]
        parsed_locations = [location.strip() for location in parsed_locations]

        parsed_placement_locations = [location for location in parsed_locations if location.endswith('(p)')]
        parsed_locations = [location.replace('(p)', '') for location in parsed_locations]
        parsed_placement_locations = [location.replace('(p)', '') for location in parsed_placement_locations]
        parsed_placement_locations = [location.strip() for location in parsed_placement_locations]
        parsed_locations = [location.strip() for location in parsed_locations]

        if parsed_locations:
            return parsed_locations, parsed_placement_locations
        else:
            warnings.warn("List of locations is empty. Check content of location markdown file")
            return []

    def parse_rooms(self, data):
        parsed_rooms = re.findall(r'\|\s*(\w+ \w*)\s*\|', data, re.DOTALL)
        parsed_rooms = [rooms.strip() for rooms in parsed_rooms]

        if parsed_rooms:
            return parsed_rooms[1:]
        else:
            warnings.warn("List of rooms is empty. Check content of room markdown file")
            return []

    def parse_objects(self, data):
        parsed_objects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
        parsed_objects = [objects for objects in parsed_objects if objects != 'Objectname']
        parsed_objects = [objects.replace("_", " ") for objects in parsed_objects]
        parsed_objects = [objects.strip() for objects in parsed_objects]

        parsed_categories = re.findall(r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
        parsed_categories = [category.strip() for category in parsed_categories]
        parsed_categories = [category.replace('(', '').replace(')', '').split() for category in parsed_categories]
        parsed_categories_plural = [category[0] for category in parsed_categories]
        parsed_categories_plural = [category.replace("_", " ") for category in parsed_categories_plural]
        parsed_categories_singular = [category[1] for category in parsed_categories]
        parsed_categories_singular = [category.replace("_", " ") for category in parsed_categories_singular]

        if parsed_objects or parsed_categories:
            return parsed_objects, parsed_categories_plural, parsed_categories_singular
        else:
            warnings.warn("List of objects or object categories is empty. Check content of object markdown file")
            return []

    def read_cmd_gen_elements_md(self):
        print(f'IN READ CMD GEN MD ELEMENTS!')

        self.common_files = 'scripts/behavior/Eindhoven2024'

        self.common_files_path = os.path.join(self.repo_path, self.common_files)

        names_file_path = os.path.join(self.common_files_path, 'names', 'names.md')
        locations_file_path = os.path.join(self.common_files_path, 'maps', 'location_names.md')
        rooms_file_path = os.path.join(self.common_files_path, 'maps', 'room_names.md')
        objects_file_path = os.path.join(self.common_files_path, 'objects', 'test.md')
        questions_file_path = os.path.join(self.common_files_path, 'questions', 'questions.md')

        names_data = self.read_data(file_path=names_file_path)
        locations_data = self.read_data(file_path=locations_file_path)
        rooms_data = self.read_data(file_path=rooms_file_path)
        objects_data = self.read_data(file_path=objects_file_path)
        questions_data = self.read_data(file_path=questions_file_path)

        # print(f'names_data: {names_data}')
        # print(f'locations_data: {locations_data}')
        # print(f'rooms_data: {rooms_data}')
        # print(f'objects_data: {objects_data}')

        names = self.parse_names(data=names_data)
        location_names, placement_location_names = self.parse_locations(data=locations_data)
        room_names = self.parse_rooms(data=rooms_data)
        object_names, object_categories_plural, object_categories_singular = self.parse_objects(data=objects_data)

        gestures = ["waving person", "person raising their left arm", "person raising their right arm", "person pointing to the left", "person pointing to the right"]

        # DEBUG
        print()
        print(f'names: {names}')
        print(f'location_names: {location_names}')
        print(f'placement_location_names: {placement_location_names}')
        print(f'room_names: {room_names}')
        print(f'object_names: {object_names}')
        print(f'object_categories_plural: {object_categories_plural}')
        print(f'object_categories_singular: {object_categories_singular}')
        print(f'gestures: {gestures}')

        all_location_names = location_names + placement_location_names

        print(f'all_location_names: {all_location_names}')

        self.gestures_action = []
        self.rooms_action = []
        self.locations_action = []
        self.names_action = []
        self.objects_categories_action = []
        self.objects_action = []

        for elem in gestures: self.gestures_action.append(elem.lower())
        self.gestures_action = list(set(self.gestures_action))

        for elem in room_names: self.rooms_action.append(elem.lower())
        self.rooms_action = list(set(self.rooms_action))

        for elem in all_location_names: self.locations_action.append(elem.lower())
        self.locations_action = list(set(self.locations_action))

        for elem in names: self.names_action.append(elem.lower())
        self.names_action = list(set(self.names_action))

        # for elem in object_categories_singular: self.objects_categories_action.append(elem.lower())
        for elem in object_names: self.objects_categories_action.append(elem.lower())
        self.object_categories_action = list(set(self.objects_categories_action))

        for elem in object_names: self.objects_action.append(elem.lower())
        self.objects_action = list(set(self.objects_action))

        self.locations_action.extend(self.rooms_action)

        self.additional_names = ['person']
        self.names_action.extend(self.additional_names)

        # DEBUG
        print()
        print(f'self.gestures_action: {self.gestures_action}')
        print(f'self.rooms_action: {self.rooms_action}')
        print(f'self.locations_action: {self.locations_action}')
        print(f'self.names_action: {self.names_action}')
        print(f'self.object_categories_action: {self.object_categories_action}')
        print(f'self.objects_action: {self.objects_action}')

    def read_cmd_gen_elements(self):
        print(f'IN READ CMD GEN ELEMENTS!')

        # For General GPSR
        # self.common_files = 'scripts/behavior/GPSRCmdGen/CommonFiles'

        # For Bordeaux2023 GPSR
        self.common_files = 'scripts/behavior/Eindhoven24/cmdgen'

        self.common_files_path = os.path.join(self.repo_path, self.common_files)

        # DEBUG
        # print('Common Files path:', self.common_files_path)

        gesture_file_name = 'Gestures.xml'
        locations_file_name = 'Locations.xml'
        names_file_name = 'Names.xml'
        objects_file_name = 'Objects.xml'
        questions_file_name = 'Questions.xml'

        gesture_file_path = os.path.join(self.common_files_path, gesture_file_name)
        locations_file_path = os.path.join(self.common_files_path, locations_file_name)
        names_file_path = os.path.join(self.common_files_path, names_file_name)
        objects_file_path = os.path.join(self.common_files_path, objects_file_name)
        questions_file_path = os.path.join(self.common_files_path, questions_file_name)

        print('-' * 50)

        self.gestures = self.read_elements('gestures', gesture_file_path)
        self.rooms = self.read_elements('rooms', locations_file_path)
        self.locations = self.read_elements('locations', locations_file_path)
        self.names = self.read_elements('names', names_file_path)
        self.objects_categories = self.read_elements('objects_categories', objects_file_path)
        self.objects = self.read_elements('objects', objects_file_path)
        self.questions = self.read_elements('questions', questions_file_path)

        print('-' * 50)

        # DEBUG
        print('gestures:', self.gestures)
        print('locations:', self.locations)
        print('names:', self.names)
        print('objects_categories:', self.objects_categories)
        print('objects:', self.objects)
        print('questions:', self.questions)

        self.gestures_action = []
        self.rooms_action = []
        self.locations_action = []
        self.names_action = []
        self.objects_categories_action = []
        self.objects_action = []

        for elem in self.gestures: self.gestures_action.append(elem['name'])

        self.gestures_action = list(set(self.gestures_action))

        for elem in self.rooms: self.rooms_action.append(elem['name'])

        self.rooms_action = list(set(self.rooms_action))

        for elem in self.locations: self.locations_action.append(elem['name'])

        self.locations_action = list(set(self.locations_action))

        for elem in self.names: self.names_action.append(elem['name'])

        self.names_action = list(set(self.names_action))
        
        # self.objects_categories_action = list(set(self.objects_categories))

        self.objects_categories_action = self.objects_categories
        
        for elem in self.objects: self.objects_action.append(elem['name'])

        self.objects_action = list(set(self.objects_action))

        # DEBUG
        print()
        print(f'self.gestures_action: {self.gestures_action}')
        print(f'self.rooms_action: {self.rooms_action}')
        print(f'self.location_action: {self.locations_action}')
        print(f'self.names_action: {self.names_action}')

        # Add additional elements necessary but not present in xml(s).

        # For General GPSR
        # self.additional_locations = ['corridor', 'bedroom', 'dining room', 'living room', 'kitchen']

        # For Bordeaux GPSR
        # self.additional_locations = ['bedroom', 'study', 'living room', 'kitchen']

        self.locations_action.extend(self.rooms_action)

        self.additional_names = ['person']
        self.names_action.extend(self.additional_names)

        # DEBUG
        print('-' * 50)
        print('locations_action:', self.locations_action)
        print('names_action:', self.names_action)
        print('objects_categories_action:', self.objects_categories_action)
        print('objects_action:', self.objects_action)
        print('gestures action: ', self.gestures_action)


    def __init__(self, real_robot=True):
        # DEBUG
        # print('-'*50)
        # print('rc23_gpsr init')
        # print('-'*50)

        # General vars

        # Config vars

        self.repo_path = rp.get_path('robocup_hsr')

        # self.read_cmd_gen_elements()
        self.read_cmd_gen_elements_md()
        self.establish_config()

        # ROS vars
        # Robot vars
        # if real_robot:
        #     self.robot = hsrb_interface.robot.Robot()
        #     self.tts = self.robot.get('default_tts')
        #     self.tts.language = self.tts.ENGLISH

        self.robot = Robot()
        self.omni_base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.try_get('gripper')

        self.ra = action_robot.RobotWithAction()

        # self.m = sr.Microphone()
        # self.r = sr.Recognizer()

        # YOLO vars
        self.classes = None
        self.detections_data = None

        # Instruction vars
        self.confirmed = False
        self.instruction_text = ''
        self.confirmation_text = ''
        self.task_count = 1
        self.conjunction_words = ['and', 'then']

        # DEBUG
        # result_text = gpt.GPTSuper(robot_enabled=True).single_speech()
        # print('Result text:', result_text)

        print('-'*50)
        print('robot:', self.robot)
        print('-'*50)


    def listen(self, txt):
        if txt != "":
            self.speak_wait(txt)
        result=""
        try:
            audio = audion_listen.Recorder().record_once()
            result = model.transcribe(audio)
            print(result["text"])
            os.remove(audio)
        except:
            print("issue saving the audio file")
        return result["text"]

    def respond(self, content):
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
            txt = chunk['message']['content']
            speak_this = speak_this + txt

        os.system('sudo systemctl stop ollama.service')

        self.speak_wait(speak_this)


    def speak_wait(self, s):
        nwords = len(s.split(" "))
        print(nwords)
        sperword = 0.3
        self.tts.say(s) #play audio
        rospy.sleep(nwords*sperword)

    def check_for_target(self, target_name):
        if self.detections_data:
            for d in self.detections_data:
                d_class = self.classes[str(int(d[5]))]

                if target_name == d_class:
                    return True
      
        return False

    def rotate_find(self, target_name):
        current_angle = 0.0
        # rot_increment = 1.05
        rot_increment = 0.75
        while current_angle < 2*math.pi: 
            self.omni_base.go_rel(0, 0, rot_increment)
            rospy.sleep(0.2)
            if self.check_for_target(target_name): return True
            self.whole_body.move_to_go()
            rospy.sleep(0.2)
            if self.check_for_target(target_name): return True
            current_angle += rot_increment

        return False

    # def go_to_position_abs(self, move_goal):
    #     try:
    #         self.whole_body.move_to_go()
    #         self.omni_base.go_abs(move_goal[0], move_goal[1], move_goal[2])
    #         self.whole_body.move_to_go()

    #         # DEBUG
    #         print('Reached go_to_position_abs location successfully')
    #     except Exception as e:
    #         # DEBUG
    #         print('Received exception go_to_position_abs:', e)

    def go_to_position_abs(self, move_goal):
        try:
            self.ra.move_to_go()
            self.ra.go_abs(move_goal[0], move_goal[1], move_goal[2])
            self.ra.move_to_go()

            # DEBUG
            print('Reached go_to_position_abs location successfully')
        except Exception as e:
            # DEBUG
            print('Received exception go_to_position_abs:', e)

    def go_to_position_rel(self, move_goal):
        try:
            self.whole_body.move_to_go()
            self.omni_base.go_rel(move_goal[0], move_goal[1], move_goal[2])
            self.whole_body.move_to_go()

            # DEBUG
            print('Reached go_to_position_abs location successfully')
        except Exception as e:
            # DEBUG
            print('Received exception go_to_position_rel:', e)

    def go_to_instruct_position(self):
        # DEBUG
        print('Going to instruct_position!')
        self.go_to_position_abs(self.instruct_pos)

    def listen_to_instruction(self):
        self.confirmed = False
        self.instruction_text = ''
        self.confirmation_text = ''

        while not self.confirmed:
            # DEBUG
            print('Please provide the instruction now...')

            # self.speak_wait('please provide the instruction now.')

            # rospy.sleep(1)

            self.instruction_text = self.listen('please provide the instruction now.')

            if self.instruction_text is None:
                # DEBUG
                print('I didn\'t hear anything. Please try again.')

                self.speak_wait('I didn\'t hear anything. Please try again.')

                continue

            self.instruction_text = self.instruction_text.lower()
            
            rospy.sleep(1)

            # DEBUG
            print(f'I understood: \'{self.instruction_text}\'')

            self.speak_wait(f'I understood: \'{self.instruction_text}')

            # DEBUG
            print('Is this correct? Please reply with \'yes\' or \'no\'.')

            # self.speak_wait('Is this correct? Please reply with \'yes\' or \'no\'.')

            # rospy.sleep(1)

            self.confirmation_text = self.listen('Is this correct? Please reply with \'yes\' or \'no\'.')

            rospy.sleep(1)

            if self.confirmation_text:
                self.confirmation_text = self.confirmation_text.lower()

                if 'yes' in self.confirmation_text:
                    # DEBUG
                    print('Instruction confirmed proceeding.')

                    self.speak_wait('instruction confirmed proceeding.')

                    self.confirmed = True

    def handle_conjunctions(self):
        # DEBUG
        print('In handle_conjunctions!')

        result = []

        instruction_text_split = self.instruction_text.split()

        # DEBUG
        print('instruction_text_split:', instruction_text_split)

        conjunction_present = False

        for elem in instruction_text_split:
            if elem in self.conjunction_words:
                conjunction_present = True

        if conjunction_present:

            # DEBUG
            print('conjunctions present in instruction text! Handling conjunctions...')

            conjunctions_pattern = '|'.join(fr'\b{re.escape(conjunction)}\b' for conjunction in self.conjunction_words)
            result = re.split(rf'\s*(?:,|{conjunctions_pattern})\s*', self.instruction_text)
            result = [substring.strip() for substring in result if substring.strip()]

        else:
            # DEBUG
            print('no conjunctions present in instruction text! Skipping conjunction handling...')
            result.append(self.instruction_text)

            # DEBUG
            print('handling conjunction result:', result)

        return result

    def replace_it_instances(self, text, text_nouns):
        print(f'IN REPLACE IT INSTANCES!')
        print(f'text: {text}')
        if ' it' in text:
            print('IT within text!')

            it_index = text.find('it')

            print(f"it_index: {(it_index, 'it')}")

            print(f'text_nouns: {text_nouns}')

            text_noun_indices = []
            for elem in text_nouns:
                print(f'text_nouns_sorted_elem: {elem}')

                text_noun_index = elem[0]

                print(f'text_noun_index: {text_noun_index}')

                text_noun_indices.append(text_noun_index)

            print(f'text_noun_indices: {text_noun_indices}')

            it_distances = []

            for noun_index in text_noun_indices:
                dist = noun_index - it_index
                it_distances.append(dist)

            print(f'it_distances: {it_distances}')

            before_it_distances = []
            for it_distance in it_distances:
                if it_distance < 0:
                    before_it_distances.append(it_distance)

            print(f'before_it_distances: {before_it_distances}')

            nearest_before_distance = max(before_it_distances)

            print(f'nearest_before_distance: {nearest_before_distance}')

            nearest_before_distance_index = it_distances.index(nearest_before_distance)

            print(f'nearest_before_distance_index: {nearest_before_distance_index}')

            it_replacement = text_nouns[nearest_before_distance_index]

            print(f'it_replacement: {it_replacement}')

            it_replacement_text = it_replacement[1]

            replaced_text = text.replace('it', it_replacement_text)

            print(f'replaced_text: {replaced_text}')

        else:
            replaced_text = text

        return replaced_text
    
    def replace_them_instances(self, text, text_nouns):
        print(f'IN REPLACE THEM INSTANCES!')
        print(f'text: {text}')
        if ' them' in text:
            print('THEM within text!')

            them_index = text.find('them')

            print(f"them_index: {(them_index, 'them')}")

            print(f'text_nouns: {text_nouns}')

            text_noun_indices = []
            for elem in text_nouns:
                print(f'text_nouns_sorted_elem: {elem}')

                text_noun_index = elem[0]

                print(f'text_noun_index: {text_noun_index}')

                text_noun_indices.append(text_noun_index)

            print(f'text_noun_indices: {text_noun_indices}')

            them_distances = []

            for noun_index in text_noun_indices:
                dist = noun_index - them_index
                them_distances.append(dist)

            print(f'them_distances: {them_distances}')

            before_them_distances = []
            for them_distance in them_distances:
                if them_distance < 0:
                    before_them_distances.append(them_distance)

            print(f'before_them_distances: {before_them_distances}')

            nearest_before_distance = max(before_them_distances)

            print(f'nearest_before_distance: {nearest_before_distance}')

            nearest_before_distance_index = them_distances.index(nearest_before_distance)

            print(f'nearest_before_distance_index: {nearest_before_distance_index}')

            them_replacement = text_nouns[nearest_before_distance_index]

            print(f'them_replacement: {them_replacement}')

            them_replacement_text = them_replacement[1]

            replaced_text = text.replace('them', them_replacement_text)

            print(f'replaced_text: {replaced_text}')
        else:
            replaced_text = text

        return replaced_text

    def parse_nouns_sentence(self, text):
        print(f'IN PARSE NOUNS!')
        
        instruction_dict = {}
        
        instruction_nouns = []
        instruction_names = []
        instruction_objects = []
        instruction_locations = []
        instruction_gestures = []

        # for elem in self.names_action:
        #     if elem in self.instruction_text:
        #         self.instruction_nouns.append((self.instruction_text.index(elem), elem))
        #         self.instruction_names.append((self.instruction_text.index(elem), elem))

        for elem in self.names_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                instruction_nouns.append((idx, elem))
                instruction_names.append((idx, elem))

        for elem in self.objects_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                instruction_nouns.append((idx, elem))
                instruction_objects.append((idx, elem))

        # DEBUG
        # print('locations_action:', self.locations_action)

        for elem in self.locations_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                instruction_nouns.append((idx, elem))
                instruction_locations.append((idx, elem))

        # DEBUG
        # print('instruction nouns:', self.instruction_nouns)

        for elem in self.gestures_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                instruction_nouns.append((idx, elem))
                instruction_gestures.append((idx, elem))

        # DEBUG
        # print('instruction nouns:', self.instruction_nouns)       

        # rospy.signal_shutdown()

        instruction_nouns_sorted = sorted(instruction_nouns, key = lambda x: x[0])
        instruction_names_sorted = sorted(instruction_names, key = lambda x: x[0])
        instruction_objects_sorted = sorted(instruction_objects, key = lambda x: x[0])
        instruction_locations_sorted = sorted(instruction_locations, key = lambda x: x[0])
        instruction_gestures_sorted = sorted(instruction_gestures, key = lambda x: x[0])

        # DEBUG
        # print('instruction nouns sorted:', self.instruction_nouns_sorted)
        # print('instruction names sorted:', self.instruction_names_sorted)
        # print('instruction objects sorted:', self.instruction_objects_sorted)
        # print('instruction locations sorted:', self.instruction_locations_sorted)
        # print('instruction gestures sorted:', self.instruction_gestures_sorted)

        instruction_dict = {
            'instruction_nouns_sorted' : instruction_nouns_sorted,
            'instruction_names_sorted' : instruction_names_sorted,
            'instruction_objects_sorted' : instruction_objects_sorted,
            'instruction_locations_sorted' : instruction_locations_sorted,
            'instruction_gestures_sorted' : instruction_gestures_sorted
        }

        return instruction_dict

    def parse_nouns(self):
        print(f'IN PARSE NOUNS!')
        self.instruction_nouns = []
        self.instruction_names = []
        self.instruction_objects = []
        self.instruction_locations = []
        self.instruction_gestures = []

        # for elem in self.names_action:
        #     if elem in self.instruction_text:
        #         self.instruction_nouns.append((self.instruction_text.index(elem), elem))
        #         self.instruction_names.append((self.instruction_text.index(elem), elem))

        for elem in self.names_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = self.instruction_text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                self.instruction_nouns.append((idx, elem))
                self.instruction_names.append((idx, elem))


        for elem in self.objects_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = self.instruction_text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                self.instruction_nouns.append((idx, elem))
                self.instruction_objects.append((idx, elem))

        # DEBUG
        # print('locations_action:', self.locations_action)

        for elem in self.locations_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = self.instruction_text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                self.instruction_nouns.append((idx, elem))
                self.instruction_locations.append((idx, elem))

        # DEBUG
        # print('instruction nouns:', self.instruction_nouns)

        for elem in self.gestures_action:
            indices = []
            start_index = 0

            while start_index != -1:
                start_index = self.instruction_text.find(elem, start_index)
                if start_index != -1:
                    indices.append(start_index)
                    start_index += 1

            # DEBUG
            # print('elem:', elem)
            # print('indices:', indices)

            for idx in indices:
                self.instruction_nouns.append((idx, elem))
                self.instruction_gestures.append((idx, elem))

        # DEBUG
        # print('instruction nouns:', self.instruction_nouns)       

        # rospy.signal_shutdown()

        self.instruction_nouns_sorted = sorted(self.instruction_nouns, key = lambda x: x[0])
        self.instruction_names_sorted = sorted(self.instruction_names, key = lambda x: x[0])
        self.instruction_objects_sorted = sorted(self.instruction_objects, key = lambda x: x[0])
        self.instruction_locations_sorted = sorted(self.instruction_locations, key = lambda x: x[0])
        self.instruction_gestures_sorted = sorted(self.instruction_gestures, key = lambda x: x[0])

        # DEBUG
        print(f'RESULT PARSE NOUNS!')
        print(f'-' * 50)
        print('instruction nouns sorted:', self.instruction_nouns_sorted)
        print('instruction names sorted:', self.instruction_names_sorted)
        print('instruction objects sorted:', self.instruction_objects_sorted)
        print('instruction locations sorted:', self.instruction_locations_sorted)
        print('instruction gestures sorted:', self.instruction_gestures_sorted)
        print(f'-' * 50)

    def parse_verbs(self):
        print(f'IN PARSE VERBS!')
        self.instruction_verbs = []

        # print(f'find_verbs: {self.find_verbs}')
        # print(f'deliver_verbs: {self.deliver_verbs}')
        # print(f'bring verbs: {self.bring_verbs}')

        # print(f'before self.instruction_verbs: {self.instruction_verbs}')

        for elem in self.take_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'take'))

        for elem in self.place_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'place'))

        for elem in self.deliver_verbs: 
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'deliver'))

        for elem in self.find_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'find'))

        for elem in self.bring_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'bring'))

        for elem in self.go_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'go'))

        for elem in self.talk_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'talk'))

        for elem in self.answer_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'answer'))

        for elem in self.meet_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'meet'))

        for elem in self.tell_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'tell'))

        for elem in self.greet_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'greet'))

        for elem in self.remember_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'remember'))

        for elem in self.count_verbs:
            if elem in self.instruction_text and not 'counter' in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'count'))

        for elem in self.describe_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'describe'))

        for elem in self.offer_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'offer'))

        for elem in self.follow_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'follow'))

        for elem in self.guide_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'guide'))

        for elem in self.accompany_verbs:
            if elem in self.instruction_text:
                self.instruction_verbs.append((self.instruction_text.index(elem), 'accompany'))

        print(f'after self.instruction_verbs: {self.instruction_verbs}')

        self.instruction_verbs_sorted = sorted(self.instruction_verbs, key = lambda x: x[0])

        print(f'-' * 50)
        print('instruction verbs sorted:', self.instruction_verbs_sorted)
        print(f'-' * 50)


    def handle_noun_collisions(self): 
        # Handle noun collisions

        # DEBUG 
        # print('BEFORE - self.instruction_nouns_sorted:', self.instruction_nouns_sorted)
        # print('BEFORE - self.instruction_locations_sorted:', self.instruction_locations_sorted)
        # print('BEFORE - self.instruction_objects_sorted:', self.instruction_objects_sorted)
        # print('BEFORE - self.instruction_names_sorted:', self.instruction_names_sorted)
        # print('BEFORE - self.instruction_gestures_sorted:', self.instruction_gestures_sorted)

        indices = [t[0] for t in self.instruction_nouns_sorted]
        repeated_indices = [x for x in indices if indices.count(x) > 1]

        if repeated_indices:
            # DEBUG
            print('There are one or more noun index collisions!')

            collisions = {}

            for elem in repeated_indices:
                collisions[str(elem)] = []

            for tuple_item in self.instruction_nouns_sorted:
                if tuple_item[0] in repeated_indices:
                    collisions[str(tuple_item[0])].append(tuple_item[1])

            print('collisions:', collisions)

            for key, value in collisions.items():
                # DEBUG
                print('key, value:', key, value)
                for i in range(len(value)):
                    for j in range(i + 1, len(value)):
                        original_value = value[i]

                        # DEBUG
                        print('Original value:', original_value)

                        extended_value = value[j]

                        # DEBUG
                        print('Extended value:', extended_value)
                        
                        if original_value in extended_value:

                            # DEBUG
                            print('original value in extended value!')

                            for elem in self.instruction_nouns_sorted:
                                if elem[0] == int(key) and elem[1] == original_value:
                                    self.instruction_nouns_sorted.remove(elem)
                            
                            for elem in self.instruction_locations_sorted:
                                if elem[0] == int(key) and elem[1] == original_value:
                                    self.instruction_locations_sorted.remove(elem)

                            for elem in self.instruction_objects_sorted:
                                if elem[0] == int(key) and elem[1] == original_value:
                                    self.instruction_objects_sorted.remove(elem)

                            for elem in self.instruction_names_sorted:
                                if elem[0] == int(key) and elem[1] == original_value:
                                    self.instruction_names_sorted.remove(elem)
                            
                            for elem in self.instruction_gestures_sorted:
                                if elem[0] == int(key) and elem[1] == original_value:
                                    self.instruction_gestures_sorted.remove(elem)

        # DEBUG
        # print('AFTER - self.instruction_nouns_sorted:', self.instruction_nouns_sorted)
        # print('AFTER - self.instruction_locations_sorted:', self.instruction_locations_sorted)
        # print('AFTER - self.instruction_objects_sorted:', self.instruction_objects_sorted)
        # print('AFTER - self.instruction_names_sorted:', self.instruction_names_sorted)
        # print('AFTER - self.instruction_gestures_sorted:', self.instruction_gestures_sorted)

    def handle_verb_collisions(self):
        # Handle verb collisions
        indices = [t[0] for t in self.instruction_verbs_sorted]
        repeated_indices = [x for x in indices if indices.count(x) > 1]

        if repeated_indices:
            # DEBUG
            print('There are one or more verb index collisions!')

            collisions = {}

            for elem in repeated_indices:
                collisions[str(elem)] = []

            for tuple_item in self.instruction_verbs_sorted:
                if tuple_item[0] in repeated_indices:
                    collisions[str(tuple_item[0])].append(tuple_item[1])

            print('collisions:', collisions)

            for key, value in collisions.items():
                print(key, value)

                nouns_after = [x for x in self.instruction_nouns_sorted if int(x[0]) > int(key)]

                noun_after = nouns_after[0]

                # DEBUG
                print('noun after:', noun_after)

                # Produced by 'take' verb.
                if 'guide' in value and 'take' in value:
                    # DEBUG
                    print('resolve guide and take collision.')

                    if noun_after in self.instruction_names_sorted:
                        # DEBUG
                        print('noun after is name!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'take':
                                self.instruction_verbs_sorted.remove(elem)
                    
                    elif noun_after in self.instruction_objects_sorted:
                        # DEBUG
                        print('noun after is object!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'guide':
                                self.instruction_verbs_sorted.remove(elem)

                # Produced by 'deliver' verb.
                elif 'deliver' in value and 'bring' in value:
                    # DEBUG
                    print('resolve deliver and bring collision.')

                    if noun_after in self.instruction_names_sorted:
                        # DEBUG
                        print('noun after is name!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'deliver':
                                self.instruction_verbs_sorted.remove(elem)
                    
                    elif noun_after in self.instruction_objects_sorted:
                        # DEBUG
                        print('noun after is object!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'bring':
                                self.instruction_verbs_sorted.remove(elem)

                # Produced by 'count' verb.
                elif 'count' in value and 'tell' in value and 'describe' in value:
                    # DEBUG
                    print('resolve count and tell collision.')

                    if "tell me how many" in self.instruction_text:
                        # DEBUG
                        print('tell me how many - count prompt!')
                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'tell':
                                self.instruction_verbs_sorted.remove(elem)
                            if elem[0] == int(key) and elem[1] == 'describe':
                                self.instruction_verbs_sorted.remove(elem)

                    elif "tell me how" in self.instruction_text:
                        # DEBUG
                        print('tell me how - describe prompt!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'tell':
                                self.instruction_verbs_sorted.remove(elem)
                        if elem[0] == int(key) and elem[1] == 'count':
                                self.instruction_verbs_sorted.remove(elem)

                # Produced by 'greet' verb.
                elif 'greet' in value and 'talk' in value:
                    # DEBUG
                    print('resolve greet and talk collision.')

                    if "say hello to" in self.instruction_text:
                        # DEBUG
                        print('say hello to - greet prompt!')
                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'talk':
                                self.instruction_verbs_sorted.remove(elem)

                    elif "say" in self.instruction_text:
                        # DEBUG
                        print('say/talk prompt!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'greet':
                                self.instruction_verbs_sorted.remove(elem)
                    


                # Produced by 'go' and 'go behind' or go after verb. 
                elif 'follow' in value and 'go place' in value:
                    # DEBUG
                    print('resolve follow and go place collision.')

                    if noun_after in self.instruction_names_sorted:
                        # DEBUG
                        print('noun after is name!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'go place':
                                self.instruction_verbs_sorted.remove(elem)
                    
                    elif noun_after in self.instruction_locations_sorted:
                        # DEBUG
                        print('noun after is object!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'follow':
                                self.instruction_verbs_sorted.remove(elem)

                # Case produced by 'accompany', which is too vague.
                elif 'follow' in value and 'guide' in value:
                    # DEBUG
                    print('resolve follow and guide collision.')

                    # For now, choosing guide instead of follow but better solution needed.
                    for elem in self.instruction_verbs_sorted:
                        if elem[0] == int(key) and elem[1] == 'follow':
                            self.instruction_verbs_sorted.remove(elem)

                elif 'take' in value and 'meet' in value:
                    # DEBUG
                    print('resolve take and meet collision.')

                    if noun_after in self.instruction_names_sorted:
                        # DEBUG
                        print('noun after is name!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'take':
                                self.instruction_verbs_sorted.remove(elem)
                    
                    elif noun_after in self.instruction_objects_sorted:
                        # DEBUG
                        print('noun after is object!')

                        for elem in self.instruction_verbs_sorted:
                            if elem[0] == int(key) and elem[1] == 'meet':
                                self.instruction_verbs_sorted.remove(elem)

                else:
                    print('Collision present but not currently resolved.')

        print('self.instruction_verbs_sorted:', self.instruction_verbs_sorted)

    def find_closest_person_after(self, start_index):
        min_diff = float('inf')
        closest_person_entry = None

        for elem in self.instruction_names_sorted:
            index = elem[0]

            # Condition ensures we only look forward.
            if index > start_index:
                diff = abs(start_index - index)
                if diff < min_diff:
                    min_diff = diff
                    closest_person_entry = elem

        return closest_person_entry
    
    def find_closest_object_after(self, start_index):
        min_diff = float('inf')
        closest_object_entry = None

        for elem in self.instruction_objects_sorted:
            index = elem[0]

            # Condition ensures we only look forward.
            if index > start_index:
                diff = abs(start_index - index)
                if diff < min_diff:
                    min_diff = diff
                    closest_object_entry = elem

        return closest_object_entry
    
    def find_closest_location_after(self, start_index):
        min_diff = float('inf')
        closest_location_entry = None

        for elem in self.instruction_locations_sorted:
            index = elem[0]

            # Condition ensures we only look forward.
            if index > start_index:
                diff = abs(start_index - index)
                if diff < min_diff:
                    min_diff = diff
                    closest_location_entry = elem

        return closest_location_entry

    def find_closest_gesture_after(self, start_index):
        min_diff = float('inf')
        closest_location_entry = None

        for elem in self.instruction_gestures_sorted:
            index = elem[0]

            # Condition ensures we only look forward.
            if index > start_index:
                diff = abs(start_index - index)
                if diff < min_diff:
                    min_diff = diff
                    closest_location_entry = elem

        return closest_location_entry
    
    def repeat_task(self):
        if self.task_count == 1: self.first_task()
        elif self.task_count == 2: self.second_task()
        elif self.task_count == 3: self.third_task()
    
    def handle_invalid_person_name(self):
        print(f'Invalid person name provided! Please dictate a correct instruction...')
        self.speak_wait('invalid person name provided. please dictate a correct instruction.')
        self.repeat_task()

    def handle_invalid_entity(self):
        print(f'Invalid entity provided! Please dictate a correct instruction...')
        self.speak_wait('invalid entity provided. please dictate a correct instruction.')
        self.repeat_task()

    def handle_invalid_location(self):
        print(f'Invalid location provided! Please dictate a correct instruction...')
        self.speak_wait('invalid location provided. please dictate a correct instruction.')
        self.repeat_task()


    def take(self, verb):
        # DEBUG
        print('take function!')

        find = self.find(verb)

        # This is what find looks like if you find the object.
        # (True, entity_status, find_entry, location_entry)

        # This is what find looks like if you DONT find the object.
        # (False, entity_status, find_entry, location_entry)

        find_status = find[0]

        if find_status:
            entity_status = find[1]
            if entity_status == 'object':
                find_entry = find[2]

                find_index = find_entry[0]
                find_name = find_entry[1]

                object_to_take = find_name

                # Starting up list_map_objects
                rospy.wait_for_service('list_map_objects')
                _list_map_objects = rospy.ServiceProxy('list_map_objects', list_map_objects)

                print('calling list_map_objects')
                resp = _list_map_objects()

                all_object_names = resp.name
                if (len(resp.name) == 0):
                    i = 0
                    timeout = 50
                    while((len(resp.name) == 0) or (i > timeout)):
                        i = i + 1
                        resp = _list_map_objects()
                        print('no objects found')

                # DEBUG
                print('object_to_take:', object_to_take)
                print('all_object_names:', all_object_names)

                if (object_to_take not in all_object_names):
                    print(object_to_take, 'not found. requesting operator assistance')
                    self.speak_wait(f'Operator, please place {object_to_take} in my gripper')
                    rospy.sleep(1)
                    self.speak_wait('Closing gripper in 5 seconds')
                    rospy.sleep(5)
                    self.gripper.apply_force(1)
                else:
                    # Starting up pick_map_object
                    rospy.wait_for_service('pick_map_object')
                    _pick_map_object = rospy.ServiceProxy('pick_map_object', pick_map_object)

                    self.whole_body.move_to_go()
                    resp = _pick_map_object(object_to_take, robocup_hsr.msg.PickGoal().NONE, 0)

                    errors = {0: "Success!", 1:"Pick failure", 2:"Map object not found", 3:"Map object picked already", 4:"failed pick check"}

                    print('Number of objects on the table:', len(all_object_names))
                    code = errors[resp.error_code]

                    print(code)

                    if resp.error_code != 0:
                        print('failed to grab', object_to_take, '. requesting operator assistance')
                        self.speak_wait(f'Operator, please place {object_to_take} in my gripper')
                        rospy.sleep(1)
                        self.speak_wait('Closing gripper in 5 seconds')
                        rospy.sleep(5)
                        self.gripper.apply_force(1)

    def test_take(self):
        location_given = True
        
        # Comment/Uncomment based on which test(s) to perform.

        # Test singular command.
        element = 'pear'

        self.instruction_text = f'Pick up {element}.'

        self.parse_instruction()

        rospy.sleep(3)
        
        self.go_to_instruct_position()

        # Test for taking object.
        # for object in self.objects_action:
        #     self.instruction_text = f'Pick up {object}.'

        #     self.parse_instruction()

        #     rospy.sleep(3)
            
        #     self.go_to_instruct_position()

    def place(self, verb):
        print('place function!')
        
        verb_index = verb[0]
        location_entry = self.find_closest_location_after(verb_index)
        go_place_location_name = location_entry[1]

        pos = self.place_loc[go_place_location_name]
        print("approaching shelves")
        self.omni_base.go_abs(pos[0], pos[1], pos[2])
        self.ra.move_to_go()

        self.ra.move_head(self.whole_body.joint_positions['head_pan_joint'], -0.3, True)
        
        rospy.wait_for_service('place_map_object')
        _place_map_object = rospy.ServiceProxy('place_map_object', place_map_object)

        # logic for pic
        if pos[-1] == 'FRONT':
            object_pick_type = robocup_hsr.msg.PickGoal().FRONT

        loc = PoseStamped()
        loc.header.frame_id = '/map'
        loc.pose.position.x = pos[3]
        loc.pose.position.y = pos[4]
        loc.pose.position.z = pos[5]
        loc.pose.orientation.x = pos[6]
        loc.pose.orientation.y = pos[7]
        loc.pose.orientation.z = pos[8]
        loc.pose.orientation.w = pos[9]
        self.ra.move_to_neutral()
        
        resp = _place_map_object(loc, object_pick_type)

        errors = {0: "Success!", 1:"Place failure", 2:"Map object not found", 3:"Map object picked already"}

        code = errors[resp.error_code]

    def pour(self, verb):
        print('pour function!')

    def deliver(self, verb):
        # DEBUG
        self.bring(verb)
        print('deliver function!')

    def bring(self, verb, separate_instructions):
        # DEBUG
        bring_idx = -1
        take_ind = -1
        count = 0

        for instruction in separate_instructions:
            if "take" in instruction or "get" in instruction or "grasp" in instruction or "fetch" in instruction:
                take_idx = count
            if "bring" in instruction or "deliver" in instruction or "give" in instruction:
                if " me " in instruction:
                    bring_idx = count
            count = count + 1

        if take_idx != -1:
            print("Already picked up the object")
        else:
            print("We need to find and grab the item first")
            self.take(verb)

        if bring_idx != -1: # bring to instruction person
            self.go_to_position_abs(self.instruct_pos)
        else:
            self.go_place(verb)
            
    def go(self, location):
        # DEBUG

        # TODO: diff between go place and go person
        self.go_place(location)

        print('go function!')
   
    def go_to(self, location):
        go_place_location_name = location
        print("This is the location I got: ")
        print(go_place_location_name)

        # This check is against yaml file data (makes sure we have a pose to go to).
        if go_place_location_name in self.locations_names:
            go_place_location_pose = eval(self.locations_pos[go_place_location_name])

            # DEBUG
            print(f'Proceeding to \'{location}\' location pose...')

            self.go_to_position_abs(go_place_location_pose)
        else: self.handle_invalid_location()

    def go_place(self, verb):
        # DEBUG
        print('go_place function!')

        # Where do we need to go?
        verb_index = verb[0]
        go_place_entry = self.find_closest_location_after(verb_index)

        # DEBUG
        print('go_place_entry:', go_place_entry)

        # sys.exit()

        # This check is against xml (generator) file data.
        if not go_place_entry: self.handle_invalid_location()
        else:
            go_place_location_name = go_place_entry[1]

            # DEBUG
            print(f'Valid xml (generator) location: \'{go_place_location_name}\'.')

            # This check is against yaml file data (makes sure we have a pose to go to).
            if go_place_location_name in self.locations_names:
                print(f'self.locations_pos[go_place_location_name]: {self.locations_pos[go_place_location_name]}')
                go_place_location_pose = eval(self.locations_pos[go_place_location_name])

                # DEBUG
                print(f'Valid yaml location pose: \'{go_place_location_pose}\'')

                # DEBUG
                print('go_place_locations_pose:', type(go_place_location_pose))
                print('go_place_location_pose:', go_place_location_pose)
               
                # DEBUG
                print(f'Proceeding to primary location pose...')

                self.go_to_position_abs(go_place_location_pose)

                # Get sub-location within room (if there is one).

                sub_go_place_index = go_place_entry[0] + 1

                print('sub_go_place_index:', sub_go_place_index)

                print('instructions_locations_sorted:', self.instruction_locations_sorted)

                sub_go_place_entry = self.find_closest_location_after(sub_go_place_index)

                if sub_go_place_entry:
                    print('sub go_place_entry:', sub_go_place_entry[0], sub_go_place_entry)

                    if sub_go_place_entry != go_place_entry:
                        sub_go_place_location_name = sub_go_place_entry[1]

                        if sub_go_place_location_name in self.locations_names:
                            sub_go_place_location_pose = eval(self.locations_pos[sub_go_place_location_name])

                            # DEBUG
                            print(f'Proceeding to secondary location pose...')

                            self.go_to_position_abs(sub_go_place_location_pose)
                        else: 
                            self.handle_invalid_location()

            else: self.handle_invalid_location()

    def go_person(self, verb):
        # DEBUG
        print('go_place function!')

        # Where do we need to go?
        verb_index = verb[0]
        go_place_entry = self.find_closest_location_after(verb_index)
        go_person_entry = self.find_closest_person_after(verb_index)

        # DEBUG
        print('go_place_entry:', go_place_entry)

        # sys.exit()

        if not go_place_entry: self.handle_invalid_person_name()
        # else we need to check if we got location as well and then look for the person

        # This check is against xml (generator) file data.
        if not go_place_entry: self.handle_invalid_location()
        else:
            go_place_location_name = go_place_entry[1]

            # DEBUG
            print(f'Valid xml (generator) location: \'{go_place_location_name}\'.')

            # This check is against yaml file data (makes sure we have a pose to go to).
            if go_place_location_name in self.locations_names:
                go_place_location_pose = eval(self.locations_pos[go_place_location_name])

                # DEBUG
                print(f'Valid yaml location pose: \'{go_place_location_pose}\'')

                # DEBUG
                print('go_place_locations_pose:', type(go_place_location_pose))
                print('go_place_location_pose:', go_place_location_pose)
               
                # DEBUG
                print(f'Proceeding to primary location pose...')

                self.go_to_position_abs(go_place_location_pose)

            else: self.handle_invalid_location()

    def test_go_place(self):
        print(f'IN TEST GO PLACE!')
        for location_name in self.locations_names:
            # DEBUG
            print('location_name:', location_name)

            self.instruction_text = f'go to the {location_name}'

            self.parse_instruction()

            rospy.sleep(3)
            
            self.go_to_instruct_position()

    def find(self, verb):
        # DEBUG
        print('find function!')

        # What or Who do we need to find?
        find_entry = None
        entity_status = None

        verb_index = verb[0]
        find_entry_person = self.find_closest_person_after(verb_index)
        find_entry_object = self.find_closest_object_after(verb_index)

        if find_entry_person and find_entry_object:
            person_index = find_entry_person[0]
            object_index = find_entry_object[0]

            if person_index < object_index: 
                find_entry = find_entry_person
                entity_status = 'person'
            else:
                find_entry = find_entry_object
                entity_status = 'object'
        elif find_entry_person:
            # DEBUG
            print('find_entry is person!') 

            find_entry = find_entry_person
            entity_status = 'person'
        elif find_entry_object: 
            # DEBUG
            print('find entry is object!')

            find_entry = find_entry_object
            entity_status = 'object'
        else: self.handle_invalid_entity()

        print(f'find entry: {find_entry}')

        find_index = find_entry[0]
        find_name = find_entry[1]

        # Where is the entity of interest?
        location_entry = self.find_closest_location_after(find_index)

        # DEBUG
        print('find location_entry:', location_entry)

        # Case where we're not given the location.
        if not location_entry: 
            # DEBUG
            print('No location entry found, starting wander_find!')

            # We have found the entity:
            if self.wander_find(find_name):
                # DEBUG
                print('Entity of interest found!')
                
                self.speak_wait(f'found {find_name}')
                return (True, entity_status, find_entry, location_entry)
            else:
                # DEBUG
                print('Entity of interest NOT found!')

        # Case where we ARE given the location.
        else:
            location_name = location_entry[1]
            self.go_to(location_name)
            if self.rotate_find(find_name): 
                # DEBUG
                print('Entity of interest found!')
                self.speak_wait(f'found {find_name}')
                return (True, entity_status, find_entry, location_entry)
            else:
                print('Entity of interest NOT found!')

        self.speak_wait(f'did not find {find_name}')

        return (False, entity_status, find_entry, location_entry)
    
    def test_find(self):
        location_given = False

        # Comment/Uncomment based on which test(s) to perform.

        # Test for finding person.
        # for name in self.names_action:
        #     if location_given:
        #         for location in self.locations_action:
        #             self.instruction_text = f'Please look for {name} in {location}.'

        #             self.parse_instruction()

        #             rospy.sleep(3)
                    
        #             self.go_to_instruct_position()

        #     else:
        #         self.instruction_text = f'Please look for {name}.'

        #         self.parse_instruction()

        #         rospy.sleep(3)
                
        #         self.go_to_instruct_position()

        # Test for finding object.
        # for object in self.objects_action:
        #     if location_given:
        #         for location in self.locations_action:
        #             self.instruction_text = f'Please look for {object} in {location}.'

        #             self.parse_instruction()

        #             rospy.sleep(3)
                    
        #             self.go_to_instruct_position()

        #     else:
        #         self.instruction_text = f'Please look for {object}.'

        #         self.parse_instruction()

        #         rospy.sleep(3)
                
        #         self.go_to_instruct_position()

    def wander_find(self, target_name):
        preferred_location = None

        rooms_to_visit = self.rooms_action.copy()

        print(f'self.object_categories_action: {self.objects_categories_action}')

        for category in self.objects_categories_action:
            # print(f'category: {category}')
            # print(f'type category: {type(category)}')
            if isinstance(category, dict):
                if 'categoryList' in list(category.keys()):
                    print(f"category[categoryList]: {category['categoryList']}")
                    if target_name in category['categoryList']:
                        preferred_location = category['defaultLocation']
                        break

        if preferred_location:
            # DEBUG
            print(f'Going to preferred location: {preferred_location} first.')

            self.go_to(preferred_location)

            if preferred_location in rooms_to_visit:
                rooms_to_visit.remove(preferred_location)

            if self.rotate_find(target_name): 
                return True

        print(f'rooms_to_visit: {rooms_to_visit}')
            
        for room in rooms_to_visit:
            self.go_to(room)
            if self.rotate_find(target_name): 
                return True
            
        return False

    def talk(self, verb, instruction):
        # DEBUG
                # What or Who do we need to find?
        find_entry = None
        entity_status = None
        prompt = None

        verb_index = verb[0]
        find_entry_person = self.find_closest_person_after(verb_index)
        find_entry_object = self.find_closest_object_after(verb_index)

        if find_entry_person and find_entry_object:
            person_index = find_entry_person[0]
            object_index = find_entry_object[0]

            if person_index < object_index: 
                find_entry = find_entry_person
                entity_status = 'person'
            else:
                find_entry = find_entry_object
                entity_status = 'object'
        elif find_entry_person:
            # DEBUG
            print('find_entry is person!') 

            find_entry = find_entry_person
            entity_status = 'person'
        elif find_entry_object: 
            # DEBUG
            print('find entry is object!')

            find_entry = find_entry_object
            entity_status = 'object'
        else: self.handle_invalid_entity()

        if entity_status == 'person':
            prompt = "Talk about: " + find_entry_person
        elif entity_status == 'object':
            prompt = "Talk about: " + find_entry_object
        self.respond(prompt)

    def answer(self, verb):
        # DEBUG
        verb_index = verb[0]
        print('answer function!')
        target = self.find_closest_object_after(verb_index)

        # TODO: Should we check if the question has been asked at the instruction point?

        question = self.listen("What would you like me to answer?")
        self.respond(question)

    def meet(self, verb):
        # DEBUG
        print('meet function!')

        # Who/What do we need to meet?

        find = self.find(verb)
        find_status = find[0]

        if find_status:
            entity_status = find[1]
            find_entry = find[2]

            find_index = find_entry[0]
            find_name = find_entry[1]

            if entity_status == 'person': 
                # DEBUG
                print(f'hello {find_name}')
                print(f'Nice to meet you!')

                self.speak_wait(f'hello {find_name}')
                self.speak_wait(f'Nice to meet you!')

    def test_meet(self):
        location_given = True

        # Comment/Uncomment based on which test(s) to perform.

        # Test singular command.
        # element = 'person'

        # self.instruction_text = f'Get acquainted with {element}.'

        # self.parse_instruction()

        # rospy.sleep(3)
        
        # self.go_to_instruct_position()

        # Test for meet person.
        # for name in self.names_action:
        #     if location_given:
        #         for location in self.locations_action:
        #             self.instruction_text = f'Meet with {name} who can be found in {location}.'

        #             self.parse_instruction()

        #             rospy.sleep(3)
                    
        #             self.go_to_instruct_position()

        #     else:
        #         self.instruction_text = f'Meet with {name}.'

        #         self.parse_instruction()

        #         rospy.sleep(3)
                
        #         self.go_to_instruct_position()

        # Test for finding object.
        # for object in self.objects_action:
        #     if location_given:
        #         for location in self.locations_action:
        #             self.instruction_text = f'Meet with {object} who can be found in {location}.'

        #             self.parse_instruction()

        #             rospy.sleep(3)
                    
        #             self.go_to_instruct_position()

        #     else:
        #         self.instruction_text = f'Meet with {name}.'

        #         self.parse_instruction()

        #         rospy.sleep(3)
                
        #         self.go_to_instruct_position()

    def tell(self, verb):
        # DEBUG
        print('tell function!')

        entity_status = ""
        prompt = ""
        verb_index = verb[0]
        find_entry_person = self.find_closest_person_after(verb_index)
        find_entry_object = self.find_closest_object_after(verb_index)
        find_entry_location = self.find_closest_location_after(verb_index)
        find_entry_gesture = self.find_closest_gesture_after(verb_index)
        find_second_location = self.find_closest_location_after(find_entry_location[0] + 1)
        find_second_person = self.find_closest_person_after(find_entry_person[0] + 1)

        if find_entry_person and find_entry_location: # if we have something about finding person at the position
            person_index = find_entry_person[0]
            object_index = find_entry_object[0]
            self.go_person()
            self.find_person()

        if find_entry_gesture:
            if find_second_person:
                if find_second_person[0] < find_entry_gesture[0]:
                    print(" We will be looking at the pose of the second person")
            else:
                self.person_pose_gesture()


        # if we have location and no go we need to go to that location anyway




    def greet(self, verb):
        # DEBUG
        print('meet function!')

        # Who/What do we need to meet?

        find = self.find(verb)
        find_status = find[0]

        if find_status:
            entity_status = find[1]
            find_entry = find[2]

            find_index = find_entry[0]
            find_name = find_entry[1]

            if entity_status == 'person': 
                # DEBUG
                print(f'hello {find_name}')
                print(f'Nice to meet you!')

                self.speak_wait(f'hello {find_name}')
                self.speak_wait(f'Nice to meet you!')

    def remeber(self):
        # DEBUG
        print('remember function!')

    def count(self, verb, instruction):
        # DEBUG
        # parse image from a location (ex. shelve) and count the items requested


        # TODO: if we have go to and describe if we are describing an operator we need to take a pick of them before we go

        # TODO: if we need to find something and describe it to someone we need to take a pic as well
        # DEBUG
        # use the description from receptionist
        entity_status = ""
        prompt = ""
        verb_index = verb[0]
        find_entry_person = self.find_closest_person_after(verb_index)
        find_entry_object = self.find_closest_object_after(verb_index)
        location = self.find_closest_location_after(verb_index)

        if find_entry_person and find_entry_object:
            person_index = find_entry_person[0]
            location_index = find_entry_object[0]

            if person_index < object_index: 
                find_entry = find_entry_person
                entity_status = 'person'
            else:
                find_entry = find_entry_object
                entity_status = 'object'
        elif find_entry_person:
            # DEBUG
            print('find_entry is person!') 

            find_entry = find_entry_person
            entity_status = 'person'
        elif find_entry_object: 
            # DEBUG
            print('find entry is object!')

            find_entry = find_entry_object
            entity_status = 'object'
        else: self.handle_invalid_entity()

        # TODO: Diffrenciate between describe something known and something that is in front of you ex. person

        if entity_status == 'person':
            # TODO: check what person is being described
            # Currently we descrine the person we are giving description to
            prompt = "Go gpsr <image>\nUSER: Please count how many " + find_entry_person[1] + "are there. \nASSISTANT:"
        elif entity_status == 'object':
            # if we are describing somethiing we know of
            # self.tell(verb)
            
            # else if we need to find an object and describe it to a person
            prompt = "Go gpsr <image>\nUSER: Please count how many " + find_entry_object[1] + "are there. \nASSISTANT:"

        else:
            prompt = "Go gpsr <image>\nUSER: Please "  + instruction + ". \nASSISTANT:"


        photo_command_pub.publish(prompt) 

        while not os.path.exists('/home/robocanes/hsr_robocanes/thing_person_description.txt'):
            x = 1
        with open("/home/robocanes/hsr_robocanes/thing_person_description.txt", "r") as file:
            description = file.read()
            print(description)
            self.speak_wait(description)

        print('count function!')

    def describe(self, verb, instruction):
        global img_name, img_num, img_list, exit_capture, go_mark, photo_snap #,cvbridge

        # TODO: if we have go to and describe if we are describing an operator we need to take a pick of them before we go

        # TODO: if we need to find something and describe it to someone we need to take a pic as well
        # DEBUG
        # use the description from receptionist
        entity_status = ""
        prompt = ""
        verb_index = verb[0]
        find_entry_person = self.find_closest_person_after(verb_index)
        find_entry_object = self.find_closest_object_after(verb_index)

        if find_entry_person and find_entry_object:
            person_index = find_entry_person[0]
            object_index = find_entry_object[0]

            if person_index < object_index: 
                find_entry = find_entry_person
                entity_status = 'person'
            else:
                find_entry = find_entry_object
                entity_status = 'object'
        elif find_entry_person:
            # DEBUG
            print('find_entry is person!') 

            find_entry = find_entry_person
            entity_status = 'person'
        elif find_entry_object: 
            # DEBUG
            print('find entry is object!')

            find_entry = find_entry_object
            entity_status = 'object'
        else: self.handle_invalid_entity()

        # TODO: Diffrenciate between describe something known and something that is in front of you ex. person

        if entity_status == 'person':
            # TODO: check what person is being described
            # Currently we descrine the person we are giving description to
            prompt = "Go gpsr <image>\nUSER: Please describe " + find_entry_person[1] + ". \nASSISTANT:"
        elif entity_status == 'object':
            # if we are describing somethiing we know of
            # self.tell(verb)
            
            # else if we need to find an object and describe it to a person
            prompt = "Go gpsr <image>\nUSER: Please describe " + find_entry_object[1] + ". \nASSISTANT:"

        else:
            prompt = "Go gpsr <image>\nUSER: "  + instruction + ". \nASSISTANT:"


        photo_command_pub.publish(prompt) 

        while not os.path.exists('/home/robocanes/hsr_robocanes/thing_person_description.txt'):
            x = 1
        with open("/home/robocanes/hsr_robocanes/thing_person_description.txt", "r") as file:
            description = file.read()
            print(description)
            self.speak_wait(description)

    def offer(self):
        # DEBUG
        # likely offer some sort of item
        # will 
        print('offer function!')

    def follow(self):
        # DEBUG
        # here we need something similar to carry my luggage but just the following part
        print('follow function!')

    def guide(self, verb):
        # DEBUG
        self.speak_wait("Please, follow me.")
        self.go_to(verb)
        print('guide function!')

        # Who do we need to guide?
        # verb_index = verb[0]
        # guide_person_entry = self.find_closest_person_after(verb_index)

        # # DEBUG
        # print('guide_person_entry:', guide_person_entry)
        
        # # if not guide_person_entry: self.handle_invalid_person_name()
        # # else:
        # #     guide_person = guide_person_entry[1]
        # #     # DEBUG
        # #     print('guide person:', guide_person)

        # # Where do we need to guide this person?
        # person_index = guide_person_entry[0]
        # to_location_entry = self.find_closest_location_after(person_index)

        # if not to_location_entry: self.handle_invalid_location()
        # else:
        #     to_location = to_location_entry[1]

        #     # DEBUG
        #     print('to_location:', to_location)

        # Where is this person found?

    def center_on_person_found(self):
        global keypoints
        # TODO: use the points from openpose to orient head
        # so we are looking at the person
        # we can consider passing the lef/right ankle pixels

        left_ankle = keypoints[14].pixel  # Left ankle
        right_ankle = keypoints[11].pixel  # Right ankle

        midpoint = ((left_ankle.pixel.x + right_ankle.pixel.x) / 2, (left_ankle.pixel.y + right_ankle.pixel.y) / 2)
        # x = 640
        # y = 480
        diff_x = 320 - midpoint[0]
        diff_y = 240 - midpoint[1]

        while abs(diff_x > 10):
            if diff_x > 0:
                ra.move_head(whole_body.joint_positions["head_pan_joint"]+0.1, whole_body.joint_positions["head_tilt_joint"])
            else:
                ra.move_head(whole_body.joint_positions["head_pan_joint"]+0.1, whole_body.joint_positions["head_tilt_joint"])

            rospy.sleep(1) 

            midpoint = ((left_ankle.pixel.x + right_ankle.pixel.x) / 2, (left_ankle.pixel.y + right_ankle.pixel.y) / 2)
            # x = 640
            # y = 480
            diff_x = 320 - midpoint[0]
            diff_y = 240 - midpoint[1]

    def find_person(self):
        global keypoints
        left_ankle = 0
        right_ankle = 0
        rospy.sleep(1.)

        left_ankle = keypoints[14].pixel  # Left ankle
        right_ankle = keypoints[11].pixel  # Right ankle

        count = 0

        found_person = True
        
        whole_body.move_to_joint_positions({'head_tilt_joint': -0.5})
        rospy.sleep(4.)
        while count < 7:
            if(left_ankle == 0 or right_ankle == 0):
                omni_base.go_rel(0, 0, -0.7)
                count = count + 1
                if(count == 6):
                    found_person = False
            elif (left_ankle.pixel.x == 0 and left_ankle.pixel.y == 0 and right_ankle.pixel.x == 0 and right_ankle.pixel.y == 0):
                print("no person")
                omni_base.go_rel(0, 0, -0.7)
                count = count + 1
                if(count == 6):
                    found_person = False
            else:
                found_person = True
                self.center_on_person_found()

                count = 8
        return found_person
            

    def accompany(self):
        # DEBUG
        print('accompany function!')

    def detect_keypoints(self, msg):
        global keypoints
        for person in msg.persons: 
            keypoints =  person.bodyParts

    def recognize_action(self, keypoints):
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

    def angle(self, shoulder, hip, knee):
        a = np.array([shoulder.x, shoulder.y])
        b = np.array([hip.x, hip.y])
        c = np.array([knee.x, knee.y])

        ba = a - b
        bc = c - b

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        print(np.degrees(angle))
        return np.degrees(angle)

    def recognize_posture(self, keypoints):
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
            return 0 #"Sitting person"

        if (posture_angle_left < 30 and posture_angle_right < 30) or (posture_angle_left > 130 and posture_angle_right > 130):

            # Check for vertical vs horizontal

            if((left_hip.y < left_knee.y) and (right_hip.y < right_knee.y)):
                return 1 #"Standing person"
            if abs(left_hip.y - right_hip.y) < 30 and abs(left_knee.y - right_knee.y) < 30:
                if abs(left_hip.y - left_knee.y) < 50:
                    return 2 #"Lying person"


        return 

    def person_pose_gesture(self, action_item):
        global keypoints
        keypoints = None

        num = 0

        gestures_options = {"waving", "left arm", "right arm", "pointing to the left", " pointing to the right"}
        pose_options = {"sitting", "standing", "laying"}

        while num < 10:
            if not keypoints == None and not keypoints[11].pixel.x == 0.0: 

                if action_item in gestures_options:
                    actions = recognize_action(keypoints)
                    print(f"Detected actions: {actions}")
                if action_item in pose_options:
                    postures = recognize_posture(keypoints)
                    print(f"Detected actions: {pose_options[postures]}")
                num = num+1
    


    def parse_instruction(self):
        # Parsing received instruction.
        print('PARSING RECEIVED INSTRUCTION TEXT.')

        self.instruction_text = self.instruction_text.lower().strip().translate(str.maketrans('', '', string.punctuation.replace(',', '')))

        # DEBUG
        print('Instruction text:', self.instruction_text)

        instruction_nouns_dict = self.parse_nouns_sentence(text=self.instruction_text)

        instruction_object_nouns = instruction_nouns_dict['instruction_objects_sorted']

        instruction_person_nouns = instruction_nouns_dict['instruction_names_sorted']

        # This handles the 'it' instances by replacing with closest previous object substring.
        self.instruction_text = self.replace_it_instances(text=self.instruction_text, text_nouns=instruction_object_nouns)

        # This handles the 'them' instances by replacing with closest previous person substring.
        self.instruction_text = self.replace_them_instances(text=self.instruction_text, text_nouns=instruction_person_nouns)

        print(f'After instruction text: {self.instruction_text}')

        # sys.exit()

        seperate_instructions = self.handle_conjunctions()

        # DEBUG
        print('Instructions List:', seperate_instructions)

        for instruction in seperate_instructions:
            self.instruction_text = instruction
            sub_instructions = []

            # DEBUG
            print('start_instruction_text:', self.instruction_text)

            self.parse_nouns()
            self.parse_verbs()
            self.handle_noun_collisions()
            self.handle_verb_collisions()

            print('instruction nouns sorted:', self.instruction_nouns_sorted)
            print('instruction names sorted:', self.instruction_names_sorted)
            print('instruction objects sorted:', self.instruction_objects_sorted)
            print('instruction locations sorted:', self.instruction_locations_sorted)
            print('instruction gestures sorted:', self.instruction_gestures_sorted)

            # DEBUG
            print(f'processed_instruction_text: {self.instruction_text}')

            # DEBUG
            print('instruction_verbs_sorted:', self.instruction_verbs_sorted)

            # count verbs and nouns to figure out if we have even number in instruction

            print(f'len instruction_nouns_sorted: {len(self.instruction_nouns_sorted)}')
            print(f'len instruction_verbs_sorted: {len(self.instruction_verbs_sorted)}')

            # if(len(self.instruction_nouns_sorted) > len(self.instruction_verbs_sorted)):
            #     print("we need to make instruction pairs")

            #     if(len(self.instruction_locations_sorted) > 0):
            #         for location in self.instruction_locations_sorted:
            #             text = "go to " + location[1]
            #             sub_instructions.append((location[0], text))

            #     if(len(self.instruction_names_sorted) > 0):
            #         for person in self.instruction_names_sorted:
            #             text = "find " + person[1]
            #             sub_instructions.append((person[0], text))

            #     if(len(self.instruction_gestures_sorted) > 0):
            #         for gesture in self.instruction_gestures_sorted:
            #             text = "recognize " + gesture[1]
            #             sub_instructions.append((gesture[0], text))

            #     for verb in self.instruction_verbs_sorted:
            #         if "tell" in verb[1]:
            #             print(instruction)
            #             if "tell me" in instruction:
            #                 print("need to return to instruction point at thr end")
            #                 text = "go to instruction point"
            #                 sub_instructions.append((len(instruction), text))
            #                 text = "tell me"
            #                 sub_instructions.append((len(instruction) + 1, text))
            #             elif "to the person" in instruction:
            #                 text = "tell to the person"
            #                 sub_instructions.append((len(instruction), text))

            #     sub_instruction_sorted = sorted(sub_instructions, key = lambda x: x[0])
            #     split_index = [0]
            #     count = 0
            #     subs = self.verb_chierachy(sub_instruction_sorted)
            #     # print(subs)
            #     for subtask in subs:
            #         if "go " in subtask[1]:
            #             split_index.append(count)
            #         count = count + 1
                
            #     count = 0
            #     split_index.append(len(subs))

            #     new_order = []
            #     new_command = "\s"

            #     while count < len(split_index)-1:
            #         ind_s = split_index[count]
            #         if count != 0:
            #             ind_s = split_index[count] + 1
            #         ind_f = split_index[count+1]+1

            #         sub_list = subs[ind_s:ind_f]
            #         l_sub = len(sub_list)
            #         i = 0
            #         while i < l_sub:
            #             next_verb = min(sub_list, key=lambda x: x[0])
            #             new_order.append(next_verb)
            #             new_command = new_command + " and " + next_verb[1]
            #             sub_list.remove(next_verb)
            #             i = i + 1
            #         count = count + 1

            #     new_command = new_command.replace("\s and ", "")

            #     self.instruction_text = self.instruction_text.replace(instruction, new_command)

            #     print(self.instruction_text)

            #     # self.parse_instruction()

            # print('instruction_verbs_sorted:', self.instruction_verbs_sorted)

            for verb in self.instruction_verbs_sorted:
                verb_elem = verb[1]
                
                # DEBUG
                print('verb_elem:', verb_elem)

                if verb_elem == 'take': self.take(verb)
                elif verb_elem == 'place': self.place(verb)
                elif verb_elem == 'deliver': self.deliver()
                elif verb_elem == 'bring': self.bring(verb, seperate_instructions)
                elif verb_elem == 'go': self.go(verb)
                elif verb_elem == 'find': self.find(verb)
                elif verb_elem == 'talk': self.talk(verb, instruction)
                elif verb_elem == 'answer': self.answer(verb)
                elif verb_elem == 'meet': self.meet()
                elif verb_elem == 'tell': self.tell(instruction)
                elif verb_elem == 'greet': self.greet()
                elif verb_elem == 'remember': self.remember()                
                elif verb_elem == 'count': self.count(verb, instruction)
                elif verb_elem == 'describe': self.describe(verb, instruction)
                elif verb_elem == 'offer': self.offer()
                elif verb_elem == 'follow': self.follow()
                elif verb_elem == 'guide': self.guide()
                elif verb_elem == 'accompany': self.accompany()

    def verb_chierachy(self, subs):
        first_order = ["go"]
        second_order = ["find"]
        third_order = ["take", "place"]    
        fourth_order = ["deliver", "bring", "talk", "answer", "meet", "tell", "greet", "remember", "count", "describe", "offer", "follow", "guide", "accompany", "recognize"]

        for v in first_order:
            subs = [(1, line[1]) if v in line[1] else line for line in subs]
        for v in second_order:
            subs = [(2, line[1]) if v in line[1] else line for line in subs]
        for v in third_order:
            subs = [(3, line[1]) if v in line[1] else line for line in subs]
        for v in fourth_order:
            subs = [(4, line[1]) if v in line[1] else line for line in subs]
        # subs[:] = [[3, line[1]] if v in line[1] else line for v in third_order for line in subs]
        # subs[:] = [[4, line[1]] if v in line[1] else line for v in fourth_order for line in subs]

        print(subs)
  
        return subshandle

    def handle_task(self):
        self.listen_to_instruction()

        # DEBUG
        print('Received instruction:', self.instruction_text)

        self.parse_instruction()

        rospy.sleep(3)

        self.go_to_instruct_position()

    def first_task(self):
        # DEBUG
        print('Starting first task now.')
        self.speak_wait('starting first task now.')

        self.handle_task()

    def second_task(self):
        # DEBUG
        print('Starting second task now.')

        self.speak_wait('starting second task now.')

        self.handle_task()

    def third_task(self):
        # DEBUG
        print('Starting third task now.')

        self.speak_wait('starting third task now.')

        self.handle_task()

    def gpsr_command_test(self):

        # These are a list of commands generated by the egpsr generator to be used for robust testing. 

        # commands = ['Please look for a person standing in the living room and answer a question.',
        #             'Please look for a person standing in the kitchen and answer a question.',
        #             'Please look for an orange juice standing in the kitchen and answer a question.',
        #             ]

        # commands = ['Navigate to the dishwasher, meet Mary, and follow her']

        # commands = ['Navigate to the lab and meet Axel.']

        # commands = ['Go to the trashcan.']
        # commands = ["Look for an apple in the office then get it and bring it to the person raising their right arm in the office"]

        commands = ["Look for an apple in the office then get it"]
        
        for command in commands:
            self.instruction_text = command

            self.parse_instruction()

            rospy.sleep(3)

            self.go_to_instruct_position()

    # If you see a logging error on finished, this is an inherent python issue (https://bugs.python.org/issue26789).
    # It's only on shutdown and not critical problem.
    def finished(self):
        # DEBUG
        print('-'*50)
        print('Reached end of GPSR Task!')
        print('-'*50)

        self.speak_wait('end of gpsr task!')

        rospy.sleep(1)
        sys.exit(1)

    def begin(self):
        rospy.sleep(1)

        # DEBUG
        print('Starting the GPSR Task!')

        # self.speak_wait("")
        # tlk = "Adjusting for ambient noise please be quiet"
        # self.speak_wait(tlk)

        self.speak_wait('starting gpsr task!')
        self.go_to_instruct_position()

        # Task Sections

        # Regular Begin Tasks

        # self.first_task()
        # self.second_task()
        # self.third_task()

        # Timeout Begin Tasks
        task_complete_time = 120 # seconds

        self.run_with_timeout(self.first_task, task_complete_time, on_timeout=lambda: self.second_task())
        self.run_with_timeout(self.second_task, task_complete_time, on_timeout=lambda: self.third_task())
        self.run_with_timeout(self.third_task, task_complete_time, on_timeout=lambda: self.finished())

        # Test Behaviors Routines

        # Robust command test generated with egpsr grammar. 
        # self.gpsr_command_test()

        # Individual behavior functions.

        # self.test_go_place()
        # self.test_find()
        # self.test_meet()
        # self.test_take()

        self.finished()

    def run_with_timeout(self, func, timeout, on_timeout=None):
        # Helper function to run the given function with a timeout
        thread = threading.Thread(target=func)
        thread.start()
        thread.join(timeout)
        if thread.is_alive():
            rospy.logwarn("Timeout occurred")
            if on_timeout:
                on_timeout()
            return False
        else:
            rospy.loginfo("Function completed within the timeout")
            return True

    def yolo_bb_callback(self, boxes_data):
        # DEBUG
        # print('boxes_data:', boxes_data)

        if boxes_data:
            received_detections = boxes_data.data
            received_detections = json.loads(received_detections)

            # DEBUG
            # print('received_detections:', received_detections)
            # print('received_detections type:', type(received_detections))

            # Convert detections to np array for faster processing.
            detections_np_list = [np.array(detection, dtype=np.float32) for detection in received_detections]
            
            # DEBUG
            # print('received detections np list:', detections_np_list)

            self.detections_data = detections_np_list

            # DEBUG
            # print('self.detections_data:', self.detections_data)

    def yolo_classes_callback(self, classes_data):
        # DEBUG
        # print('classes_data:', classes_data)

        if self.classes is None and classes_data:
            self.classes = json.loads(classes_data.data)
        
        # DEBUG
        # print('self.classes:', self.classes)

    def laser_callback(self, msg):
        if(msg.ranges[482] > 0.5):
            self.move = True

    def run_nodes(self):
        rospy.Subscriber('/yolo_detect/boxes', String, self.yolo_bb_callback)
        rospy.Subscriber('/yolo_detect/classes', String, self.yolo_classes_callback)
        rospy.Subscriber("frame", Frame, self.detect_keypoints)

        rospy.Subscriber('/hsrb/base_scan', LaserScan, self.laser_callback)

        while not self.move:
            print(f'Waiting...')
            rospy.sleep(1)

        print("Door opened")
        rospy.sleep(3.)

        self.begin()        
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rc24_gpsr', anonymous=True)

    gpsr = rc23_gpsr(real_robot=True)
    
    gpsr.run_nodes()