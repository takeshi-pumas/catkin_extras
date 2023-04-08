# -*- coding: utf-8 -*-
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
    if found != ' None' and place != 'None':
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
    else:
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
    if who != ' None' and where != 'None':
        guest = [key for key, person_dict in knowledge['People'].items()
                 if person_dict.get('name') == who]
        knowledge['People'][guest[0]]['location'] = where
        knowledge['Places'][where]['occupied'] = guest[0]
        write_yaml(knowledge)
        return True
    else:
        return False


def add_guest(name, drink='No drink'):
    try:
        knowledge = read_yaml()
        guests_len = len(knowledge['People'])
        new_guest = f'Guest_{guests_len}'
        knowledge['People'][new_guest] = {
            'drink': drink, 'location': 'Waiting', 'name': name.capitalize()}
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
    knowledge['People'] = {'Guest_0': host}
    for i in range(1, len(knowledge['Places'])):
        knowledge['Places'][f'Place_{i}']['occupied'] = 'None'
    write_yaml(knowledge)

def find_host():
    knowledge = read_yaml()
    host_name = knowledge['People']['Guest_0']['name']
    host_place = knowledge['People']['Guest_0']['location']
    if host_place != 'None':
        loc = knowledge['Places'][host_place]['location'] 
        XY = [loc['x'],loc['y']]
    else:
        loc = host_place
    return host_name, loc

def find_room(pos):
    known_loc = read_yaml('/known_locations.yaml')
    rooms = ['bedroom','corridor','dining_room','kitchen','living_room']
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
