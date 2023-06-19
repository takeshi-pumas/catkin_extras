import yaml
import rospkg
import rospy
from std_msgs.msg import String


class SCENE():
    def __init__(self, knowlegde_file='/receptionist_knowledge2.yaml'):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path('config_files') + knowlegde_file
        #self.knowledge = self.load_data_from_yaml()
        rospy.Subscriber('/analyze_result', String, self.callback)
        self.last_seat_assigned = "None"  # Place_0, Place_1, Place_2
        self.active_guest = "None"  # Guest_0, Guest_1, Guest_2

    # -------YAML read and write------------
    def load_data_from_yaml(self):
        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data

    def save_data_to_yaml(self, new_knowledge):
        with open(self.file_path, 'w') as file:
            documents = yaml.dump(new_knowledge, file)

    # ------Add--------------------
    def add_guest(self, name, drink):
        data = self.load_data_from_yaml()
        guests_len = len(data['People'])
        new_guest = f'Guest_{guests_len}'
        data['People'][new_guest] = {
            'drink': drink,
            'location': 'None',
            'name': name,
            'description': 'None'}
        self.save_data_to_yaml(data)
        self.active_guest = new_guest

    def add_description(self, description):
        data = self.load_data_from_yaml()
        data['People'][self.active_guest]['description'] = description
        self.save_data_to_yaml(data)

    # ------Gets-------------------

    def get_available_seats(self):
        data = self.load_data_from_yaml()
        available_seats = []
        for place, info in data['Places'].items():
            if info['occupied'] == "None":
                available_seats.append(place)
        return available_seats

    def get_guest_by_name(self, guest_name):
        data = self.load_data_from_yaml()
        for guest, info in data['People'].items():
            if info['name'] == guest_name:
                return guest
        return 'None'

    def get_guest_seat(self, guest_name):
        data = self.load_data_from_yaml()
        for place, info in data['Places'].items():
            if info['occupied'] == guest_name:
                return place
        return 'None'

    def get_places_location(self):
        data = self.load_data_from_yaml()
        locs = []
        for place, info in data['Places'].items():
            if place != 'Place_0':
                xyt = []
                xyt.append(info['location']['x'])
                xyt.append(info['location']['y'])
                xyt.append(info['location']['theta'])
                locs.append(xyt)
        return locs

    def get_places(self):
        data = self.load_data_from_yaml()
        locs = []
        for place, info in data['Places'].items():
            locs.append(place)
        return locs

    def get_guest_description(self):
        data = self.load_data_from_yaml()
        #guest = self.get_guest_by_name(guest_name)
        #return data['People'][guest]['description']
        return data['People'][self.active_guest]['description']

    def get_host_info(self):
        data = self.load_data_from_yaml()
        loc = data['People']['Guest_0']['location'] #Place_1, Place_2, Place_3
        name = data['People']['Guest_0']['name']
        return name, loc


    # -----------Seat methods---------------
    def assign_seat_to_guest(self, seat):
        data = self.load_data_from_yaml()
        #guest = self.get_guest_by_name(guest_name)
        guest = self.active_guest
        if guest != 'None':
            guest_name = data['People'][guest]['name']
            data['Places'][seat]['occupied'] = guest_name
            data['People'][guest]['location'] = seat
            self.save_data_to_yaml(data)
            self.last_seat_assigned = seat
            return True
        else:
            return False

    def update_seat_assignment(self, guest_name, new_seat):
        data = self.load_data_from_yaml()
        place = self.get_guest_seat(guest_name)
        guest = self.get_guest_by_name(guest_name)
        if place != 'None' and guest != 'None':
            data['Places'][place]['occupied'] = 'None'
            data['Places'][new_seat]['occupied'] = guest_name
            data['People'][guest]['location'] = new_seat
            self.save_data_to_yaml(data)
            return True
        else:
            return False

    def clean_knowledge(self, host_name, host_location):
        data = self.load_data_from_yaml()
        host = data['People']['Guest_0']
        host['name'] = host_name
        host['location'] = host_location
        data['People'] = {'Guest_0': host}
        for place, info in data['Places'].items():
            if place == host_location:
                info['occupied'] = host_name
            elif place == 'Place_0':
                info['occupied'] = 'Not available'
            else:
                info['occupied'] = 'None'
        self.save_data_to_yaml(data)

    def callback(self, msg):
        takeshi_line = msg.data
        if len(takeshi_line) > 2:
            self.add_description(takeshi_line)
