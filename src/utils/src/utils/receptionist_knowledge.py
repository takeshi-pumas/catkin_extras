import yaml
import rospkg
import rospy
import random
from std_msgs.msg import String

class RECEPTIONIST:
    def __init__(self, knowledge_file='/receptionist_knowledge.yaml'):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path('config_files') + knowledge_file
        self.informacion_fiesta = self.load_data_from_yaml()
        rospy.Subscriber('/analyze_result', String, self.callback)
        self.last_seat_assigned = 'None'  # Place_0, Place_1, Place_2
        self.active_guest = 'None'  # Guest_0, Guest_1, Guest_2
        self.active_seat = 'None'

    # --- YAML read and write ---
    def load_data_from_yaml(self):
        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data

    def save_data_to_yaml(self, new_knowledge):
        with open(self.file_path, 'w') as file:
            documents = yaml.dump(new_knowledge, file)

    # --- Add ---
    def add_guest(self, name, drink = 'None'):
        self.active_guest = self.add_new_person(name, drink)
        self.active_seat = self.get_any_available_seat()

    def add_guest_description(self, description):
        self.informacion_fiesta['People'][self.active_guest]['description'] = description
        self.save_data_to_yaml(self.informacion_fiesta)
    
    def add_guest_drink(self, drink):
        self.informacion_fiesta['People'][self.active_guest]['drink'] = drink
    
    def add_new_person(self, name, drink):
        guests_len = len(self.informacion_fiesta['People'])
        guest_num = f'Guest_{guests_len}'
        self.informacion_fiesta['People'][guest_num] = {
            'drink': drink,
            'location': 'None',
            'name': name,
            'description': 'None'}
        self.save_data_to_yaml(self.informacion_fiesta)
        return guest_num

    # --- Gets ---
    def get_any_available_seat(self):
        seats = self.get_available_seats()
        if len(seats) > 0:
            return random.choice(seats)
        else:
            return 'None'

    def get_available_seats(self):
        available_seats = []
        for place, info in self.informacion_fiesta['Places'].items():
            if info['occupied'] == "None":
                available_seats.append(place)
        return available_seats

    def get_guest_by_name(self, guest_name):
        for guest, info in self.informacion_fiesta['People'].items():
            if info['name'] == guest_name:
                return guest
        return 'None'

    def get_guest_seat(self, guest_name):
        for place, info in self.informacion_fiesta['Places'].items():
            if info['occupied'] == guest_name:
                return place
        return 'None'

    def get_places_location(self):
        locs = []
        for place, info in self.informacion_fiesta['Places'].items():
            if place != 'Place_0':
                xyt = [info['location']['x'], info['location']['y'], info['location']['theta']]
                locs.append(xyt)
        return locs
    
    def get_active_seat_location(self):
        if self.active_seat != 'None':
            return True, [self.informacion_fiesta['Places'][self.active_seat]['location']['x'],
                    self.informacion_fiesta['Places'][self.active_seat]['location']['y'],
                    self.informacion_fiesta['Places'][self.active_seat]['location']['theta']]
        else:
            return False, [0,0,0]
    def get_places(self):
        return list(self.informacion_fiesta['Places'].keys())

    def get_active_guest_description(self):
        if self.active_guest != 'None':
            return self.informacion_fiesta['People'][self.active_guest]['description']
        else:
            return 'No active guest found'

    def get_host_info(self):
        host = self.informacion_fiesta['People']['Guest_0']
        return host['name'], host['location'], host['drink']
    
    def get_active_guest(self):
        return self.active_guest

    # --- Seat methods ---
    def seat_confirmation(self, guest_found = 'None'):
        #This means that there is no one on this seat and it is free to seat active guest here
        if guest_found == 'None':
            self.assign_seat_to_guest(self.active_guest, self.active_seat)
        #This means that there is someone on this seat and a new assign is needed
        else:
            #Verify if the guest found is part of our party
                #If the guest is part of our party update their seat
                #if not, add them to out party and assign them this seat
                #finally get again an available seat to active guest
            guest_num = self.get_guest_by_name(guest_found)
            isGuest = guest_num != 'None'
            if isGuest:
                self.update_seat_assignment(guest_num)
            else:
                guest_num = self.add_new_person(guest_found, drink='None')
                self.assign_seat_to_guest(guest_num, self.active_seat)
                self.active_seat = self.get_any_available_seat()


    def assign_seat_to_guest(self, guest, seat):
        #Update data
        self.informacion_fiesta['Places'][seat]['occupied'] = guest
        self.informacion_fiesta['People'][guest]['location'] = seat
        self.save_data_to_yaml(self.informacion_fiesta)

    def update_seat_assignment(self, guest_num):
        #get last known seat for guest found
        lastKnownSeat = self.informacion_fiesta['People'][guest_num]['location']

        #update data
        self.informacion_fiesta['Places'][lastKnownSeat]['occupied'] = 'None'
        self.informacion_fiesta['Places'][self.active_seat]['occupied'] = guest_num
        self.informacion_fiesta['People'][guest_num]['location'] = self.active_seat
        self.save_data_to_yaml(self.informacion_fiesta)

    def clean_knowledge(self, host_name, host_location):
        host = self.informacion_fiesta['People']['Guest_0']
        host['name'] = host_name
        host['location'] = host_location
        self.informacion_fiesta['People'] = {'Guest_0': host}
        for place, info in self.informacion_fiesta['Places'].items():
            if place == host_location:
                info['occupied'] = host_name
            elif place == 'Place_0':
                info['occupied'] = 'Not available'
            else:
                info['occupied'] = 'None'
        self.save_data_to_yaml(self.informacion_fiesta)

    def callback(self, msg):
        takeshi_line = msg.data
        if len(takeshi_line) > 2:
            self.add_guest_description(takeshi_line)
