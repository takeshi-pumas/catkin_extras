# -*- coding: utf-8 -*-
import sys
import rospkg
import yaml
from utils.grasp_utils import *
from utils.nav_utils import *
import math


def read_yaml(known_locations_file='/receptionist_knowledge2.yaml'):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('config_files') + known_locations_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content


def write_yaml(new_knowledge):
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('config_files') + \
        '/receptionist_knowledge2.yaml'
    with open(file_path, 'w') as file:
        documents = yaml.dump(new_knowledge, file)


def update_occupancy(found='None', place='None'):
    # Use: hsr found someone on a supposed empty place
    knowledge = read_yaml()
    # found is a name
    try:
        # get guest number of this someone
        guest = [key for key, person_dict in knowledge['People'].items()
                 if person_dict.get('name') == found]
        # get the seat that guest used to have
        seat = [key for key, places_dict in knowledge['Places'].items(
        ) if places_dict.get('occupied') == guest[0]]
        # change knowledge
        knowledge['People'][guest[0]]['location'] = place
        knowledge['Places'][seat[0]]['occupied'] = 'None'
        knowledge['Places'][place]['occupied'] = guest[0]
        write_yaml(knowledge)
        return True
    except Exception as e:
        print(f"Error: {e}. Line {sys.exc_info()[-1].tb_lineno}")
        knowledge['Places'][place]['occupied'] = 'someone'
        write_yaml(knowledge)
        return False


# Chat GPT generated


def update_occupancy_gpt(found='unknown', place='None'):
    # Use: hsr found someone on a supposed empty place
    knowledge = read_yaml()

    try:
        if found != 'unknown':
            guest = [key for key, person_dict in knowledge['People'].items()
                     if person_dict.get('name') == found]
        else:
            guest = ['unknown']

        seat = [key for key, places_dict in knowledge['Places'].items()
                if places_dict.get('occupied') == guest[0]]

        if guest[0] != 'unknown':
            knowledge['People'][guest[0]]['location'] = place

        knowledge['Places'][seat[0]]['occupied'] = 'none'
        knowledge['Places'][place]['occupied'] = guest[0]

        write_yaml(knowledge)
        return True
    except Exception as e:
        print(f"Error: {e}. Line {sys.exc_info()[-1].tb_lineno}")
        knowledge['Places'][place]['occupied'] = 'someone'
        write_yaml(knowledge)
        return False


def find_empty_places():
    knowledge = read_yaml()
    seat = [key for key, places_dict in knowledge['Places'].items(
    ) if places_dict.get('occupied') == 'None']
    loc = list(knowledge['Places'][seat[0]]['location'].values())
    t, x, y = loc
    return seat[0], [x, y, t]


def get_waiting_guests():
    try:
        knowledge = read_yaml()
        people = [key for key, places_dict in knowledge['People'].items(
        ) if places_dict.get('location') == 'Waiting']
        num_wait = len(people)
        waiting = knowledge['People'][people[0]]['name']
        return num_wait, waiting
    except:
        return 0, ''


def assign_occupancy(who='None', where='None'):
    # Use: hsr found a empty place for the new guest
    knowledge = read_yaml()
    try:
        if who != 'None' and where != 'None':
            guest = [key for key, person_dict in knowledge['People'].items()
                     if person_dict.get('name') == who]
            if len(guest) > 0:
                knowledge['People'][guest[0]]['location'] = where
                knowledge['Places'][where]['occupied'] = guest[0]
                write_yaml(knowledge)
                return True
            else:
                return False
    except:
        return False


def add_guest(name, drink='No drink'):
    try:
        knowledge = read_yaml()
        guests_len = len(knowledge['People'])
        new_guest = f'Guest_{guests_len}'
        knowledge['People'][new_guest] = {
            'drink': drink, 'location': 'Waiting', 'name': name, 'description': 'None'}
        write_yaml(knowledge)
        return True
    except:
        return False


def add_drink(drink):
    try:
        _, guest_name = get_waiting_guests()
        knowledge = read_yaml()
        guests_len = len(knowledge['People'])
        knowledge['People'][guest_name]['drink'] = drink
        write_yaml(knowledge)
        return True
    except:
        return False


def add_place():
    try:
        knowledge = read_yaml()
        tf_manager = TF_MANAGER()
        rospy.sleep(0.8)
        trans, rot = tf_manager.getTF(target_frame='base_link')
        _, _, theta = tf.transformations.euler_from_quaternion(rot)
        places_len = len(knowledge['Places'])
        new_place = f'Place_{places_len}'
        knowledge['Places'][new_place] = {'location': {'theta': round(theta % (
            6.283), 2), 'x': round(trans[0], 2), 'y': round(trans[1], 2)}, 'occupied': 'None'}
        write_yaml(knowledge)
        return True
    except:
        return False


def clean_knowledge():
    knowledge = read_yaml()
    host = knowledge['People']['Guest_0']
    host['location'] = 'None'
    knowledge['People'] = {'Guest_0': host}
    for i in range(1, len(knowledge['Places'])):
        knowledge['Places'][f'Place_{i}']['occupied'] = 'None'
    write_yaml(knowledge)


def find_host():
    knowledge = read_yaml()
    host_name = knowledge['People']['Guest_0']['name']
    host_place = knowledge['People']['Guest_0']['location']
    # if host_place != 'None':
    #loc = knowledge['Places'][host_place]['location']
    #XY = [loc['x'],loc['y']]
    # else:
    #    loc = host_place
    return host_name, host_place


def get_location_room(pos):
    known_loc = read_yaml('/known_locations.yaml')
    rooms = ['bedroom', 'corridor', 'dining_room', 'kitchen', 'living_room']
    min_dist = 100.0
    closest = None
    for room in rooms:
        x = known_loc[room][0]['x']
        y = known_loc[room][1]['y']
        distance = math.sqrt((x-pos[0])**2+(y-pos[1])**2)
        if distance < min_dist:
            min_dist = distance
            closest = room
    return closest


def add_description(name, description):
    knowledge = read_yaml()
    guest = [key for key, person_dict in knowledge['People'].items()
             if person_dict.get('name') == name]
    knowledge['People'][guest[0]]['description'] = description
    write_yaml(knowledge)


def get_guest_description(name):
    knowledge = read_yaml()
    guest = [key for key, person_dict in knowledge['People'].items()
             if person_dict.get('name') == name]
    return knowledge['People'][guest[0]]['description']


def get_guest_location(name):
    knowledge = read_yaml()
    guest = [key for key, person_dict in knowledge['People'].items()
             if person_dict.get('name') == name]
    return knowledge['People'][guest[0]]['location']


def return_places():
    knowledge = read_yaml()
    locs = []
    num_places = len(knowledge['Places'].keys())
    for i in range(1, num_places):
        xyt = []
        xyt.append(knowledge['Places'][f'Place_{i}']['location']['x'])
        xyt.append(knowledge['Places'][f'Place_{i}']['location']['y'])
        xyt.append(knowledge['Places'][f'Place_{i}']['location']['theta'])
        locs.append(xyt)
    return locs


def places_2_tf():
    tf_man = TF_MANAGER()
    locs = return_places()
    print(locs)
    for i, loc in enumerate(locs):
        pos = [loc[0], loc[1], 0.85]
        rot = tf.transformations.quaternion_from_euler(0.0, 0.0, loc[2])
        tf_man.pub_static_tf(pos=pos, rot=rot, point_name=f'Place_{i+1}')
        tf_man.pub_static_tf(pos=[1.0, 0, 0], rot=rot, point_name=f'Place_face{i+1}', ref=f'Place_{i+1}')
        print(f'done {i}')
