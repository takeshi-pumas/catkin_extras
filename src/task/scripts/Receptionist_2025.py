#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import numpy as np
from tf.transformations import euler_from_quaternion
# from smach_utils_receptionist import *
from common.states import WaitPushHand, WaitDoorOpen, GotoPlace
from common.navigation_functions import Navigation
from common.hsr_functions import Talker, Gaze, ArmController, SpeechRecog
from common.ros_functions import TFManager
from common.vision_functions import capture_frame, recognize_face, train_face, analyze_face
from common.logic import Receptionist

# Start Node
rospy.init_node('receptionist_task', anonymous=True)

# Initialize objects
voice = Talker()
tf_manager = TFManager()
omni_base = Navigation(locations_file='/known_locations_TID.yaml')
party = Receptionist("receptionist_knowledge_TID.yaml")
head = Gaze()
arm = ArmController(omni_base, tf_manager)
speech_recognition = SpeechRecog()

# Global variables
guest_name = ''
guest_image = None

# Enable debug flag to avoid navigation
debug = True

confirmation = ('yes', 'robot yes','yeah', 
                'correct', 'right', 'that is right', 
                'that is correct', 'that is it')

def publish_places():
    """Publish known locations to TF"""
    places, locations = party.get_places_location()
    for i in range(len(places)):
        tf_manager.publish_transform(
            position=[locations[i][0], locations[i][1], 0.0],
            rotation=[0.0, 0.0, locations[i][2]],
            child_frame=places[i],
            parent_frame="map",
            static=True
        )
        rospy.sleep(0.4)

        # pub faces tf's
        tf_manager.publish_transform(
            position=[1.0, 0.0, 1.0],
            rotation=[0.0, 0.0, 0.0],
            child_frame=places[i].replace("_", "_face"),
            parent_frame=places[i],
            static=True
        )
    rospy.sleep(0.4)


# Initial STATE: task setup (grammar, knowledge, robot position, ...)

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'aborted'])
        self.tries = 0

    def execute(self, userdata):
        global confirmation
        self.tries += 1
        rospy.loginfo('STATE : INITIAL')
        try:
            rospy.loginfo(f'Try {self.tries} of 5 attempts')

            # party.clean_knowledge(host_name = "john", host_location = "Place_3")
            publish_places()

            # INIT GRAMMAR FOR VOSK
            # Use with get_keywords_speech()
            # drinks = ['coke','juice','beer', 'water', 'soda', 'wine', 'i want a', 'i would like a']
            # drinks = ['coke','juice','milk', 'water', 'soda', 'wine', 
            #          'i want a', 'i would like a', 'tea', 'icedtea', 'cola', 'redwine', 'orangejuice', 'tropicaljuice']
            drinks = ('wine', 'beer', 'water', 'soda', 'coke', 'juice', 'iced tea', 'i want a', 'i would like a')
            names = ('my name is', 'i am','john', 'jack', 'paris', 'charlie', 'simone', 'robin', 'jane', 'jules')
            interests = ('music', 'sports', 'movies', 'books', 'art', 'travel', 'food', 'i like', 'i enjoy', 'i am interested in')
            declination = ('no','nope','not','now','robot no', 'incorrect', 'wrong', 'that is wrong', 'that is not right', 'that is not correct')       
            grammar = list(drinks + names + confirmation + declination + interests)
            print("Grammar: ", grammar)                                                            
            
            if self.tries == 1:
                speech_recognition.set_grammar(grammar)
                # set_grammar(gram)  ##PRESET DRINKS
                rospy.sleep(0.2)
                return 'succ'
            elif self.tries == 3:
                return 'failed'
        except rospy.ROSInterruptException:
            return 'aborted'
        
# Scan face STATE: Take a picture of the new guest to meet them

class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ['succ', 'failed', 'aborted'])
        self.tries = 0

    def execute(self, userdata):
        global guest_name, guest_image
        rospy.loginfo('State : Scan face')
        try:
            head.move_head(0.0, 0.3)
            voice.talk('Waiting for guest, look at me, please')

            # Get image to recognize a face
            image = capture_frame()
            recognized_names = recognize_face(image)
            #rospy.sleep(0.7)

            
            if recognized_names != None:
                # guest_name = res.Ids.ids
                guest_name = recognized_names
                guest_image = image
                return 'succ'
            else:
                return 'failed'
        except rospy.ROSInterruptException:
            return 'aborted'
        
# Decide face STATE: Decide if the new guest is known, unknown or can't be decided

class Decide_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed', 'unknown', 'aborted'])
        self.tries = 0
    def execute(self, userdata):
        global confirmation, guest_name, guest_image
        self.tries += 1
        rospy.loginfo("STATE: Decide face")

        try:
            # Check first if at least one known face was detected
            if not guest_name or guest_name == ["no_face"]:
                voice.talk('I did not see you, I will try again')
                return 'failed'
            
            print("guest_name: ", guest_name)
            
            # Si hay al menos un nombre conocido, proceder con la confirmación
            known_names = [n for n in guest_name if n != "unknown"]
            if known_names:
                name = str(known_names[0])
                voice.talk(f'I found you, I think you are {name}')
                voice.talk('Is it correct?')

                # Simulación de respuesta
                speech = speech_recognition.get_speech(6)
                print(speech)

                if not speech:
                    voice.talk("I could not hear you, let's try again. Please speak louder.")
                    return 'failed'

                if speech not in confirmation:
                    return 'unknown'
                
                else:
                    self.tries = 0
                    guest_name = name
                    party.add_guest(guest_name)
                    return 'succ'
                
            # Count "unknown" faces
            unknown_count = guest_name.count('unknown')

            # If there is only one unknown face
            if unknown_count == 1 and len(guest_name) == 1:
                voice.talk('I believe we have not met.')
                self.tries = 0
                return 'unknown'

            # Si hay más de un desconocido
            if unknown_count > 1:
                voice.talk('I detected multiple people, I cannot proceed.')
                return 'failed'

        except rospy.ROSInterruptException:
            return 'aborted'
# New face STATE: Train new guest on DB

class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed', 'aborted'])
        #self.new_name = ''
        #self.num_faces = 0
        self.tries = 0

    def execute(self, userdata):
        global confirmation, guest_name
        self.tries += 1
        rospy.loginfo('STATE : NEW_FACE')
        try:
            #If name is not recognized 3 times, guest will be registered as a "someone"
            if self.tries == 3:
                voice.talk ('I didnt undestand your name, lets continue')
                guest_name = 'someone'
                image = capture_frame()
                train_face(image, guest_name)
                # train_face(userdata.face_img, guest_name )
                party.add_guest(guest_name)
                self.tries = 0
                return 'succ'
            
            #Asking for name
            voice.talk('Please, tell me your name')
            speech = speech_recognition.get_speech(10)
            # speech = 'john'
            # in case thinks like I am , my name is . etc
            if len(speech.split(' ')) > 1: 
                name = (speech.split(' ')[-1])
            else: 
                name = speech

            if not name:
                voice.talk('Please repeat it and speak louder.')
                return 'failed'

            voice.talk(f'Is {name} your name?')
            speech = speech_recognition.get_speech(10)

            if speech not in confirmation:
                voice.talk ('lets try again')
                return 'failed'
            
            guest_name = name
            voice.talk (f'Nice to Meet You {guest_name}')
            party.add_guest(guest_name)
            image = capture_frame()
            train_face(image, guest_name)
            self.tries = 0
            return 'succ'
        except rospy.ROSInterruptException:
            return 'aborted'

# Get drink STATE: Ask guest for their favourite drink

class Get_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed', 'aborted'])
        self.tries = 0

    def analyze_and_describe(self, image):
        """Analiza la cara y genera una descripción."""
        # image = capture_frame()
        response = analyze_face(image)
        print("Features: ", response)
        features = response['features'][0]
        gender, race, emotion, age = features[0:4]
        pronoun = "he" if gender.lower() == "man" else "she"
        description = f"{pronoun} is a {age}-year-old {gender}, I believe {pronoun} is {race}, and seems to be {emotion}."
        party.add_guest_description(description)

    def execute(self, userdata):
        global confirmation, guest_name, guest_image
        self.tries += 1
        rospy.loginfo('STATE : GET DRINK')
        try:
            if self.tries == 1:
                self.analyze_and_describe(guest_image)

            elif self.tries == 3:
                voice.talk ('I am having trouble understanding you, lets keep going')
                drink = 'something'
                self.tries=0
                party.add_guest_drink(drink)

                self.analyze_and_describe(guest_image)
                
                return 'succ'
            
            #Asking for drink
            voice.talk('What would you like to drink?')
            drink = speech_recognition.get_speech(10)

            if len(drink.split(' ')) > 1: 
                drink=(drink.split(' ')[-1])
            
            rospy.sleep(0.4)

            if not drink:
                voice.talk("Sorry, couldn't hear you. Please speak louder.")
                return 'failed' 
            
            voice.talk(f'Did you say {drink}?')
            speech = speech_recognition.get_speech(10)
            if speech not in confirmation: 
                return 'failed' 

            party.add_guest_drink(drink)
            voice.talk("Okay")
            self.tries = 0
            return 'succ'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        
# Get interest STATE: Ask guest for their interest

class Get_interest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'aborted'])
        self.tries = 0
    
    def execute(self, userdata):
        rospy.loginfo('STATE : Ask guest for any interest')
        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        try:
            if self.tries == 1:
                voice.talk('which interest do you have?')
            elif self.tries > 2:
                voice.talk("Let's continue")
                return 'succ'
            
            # response = get_keywords_speech(10)
            # response = 'music'
            response = speech_recognition.get_speech(10)
            if not response:
                voice.talk("Repeat it louder please")
                return 'failed' 
            party.add_guest_interest(response)

            voice.talk("Nice")
            self.tries = 0
            return 'succ'
        except rospy.ROSInterruptException:
            return 'aborted'

class Find_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'aborted'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Scan table to find beverages')
        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')
        try:
            voice.talk(f"I will check if there is {party.get_active_guest_drink()}")
            voice.talk('Scanning table')
            head.move_head(0.0, -0.3)
            rospy.sleep(1)

            print("Party state: ", party)

            favorite_drink = party.get_active_guest_drink()
            # TODO: scan table to find drink implementation
            # res,position = get_favorite_drink_location(favorite_drink)
            res = True
            position = 'left'
            if res:
                self.tries = 0
                voice.talk(f"I found a {favorite_drink} on the {position}, take it please.")
                return 'succ'
            elif res.result.data == "not found":
                self.tries = 0
                voice.talk(f'There is no {favorite_drink}, if you want to, take another one.')
                return 'succ'
            else:
                return 'failed'
        except rospy.ROSInterruptException:
            return 'aborted'
        
# Find sitting place STATE: Find a place to sit according to previous knowledge of the world

class Find_sitting_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'aborted'])
        self.tries = 0
    def execute(self, userdata):

        rospy.loginfo('STATE : Looking for a place to sit')
        try:
            print('Try', self.tries, 'of 3 attempts')
            self.tries += 1
            voice.talk('I am looking for a place for you')
            place = party.get_active_seat()

            if place:
                tf_name = place.replace('_', '_face')
                head.look_at_frame(tf_name)

            # commented detect human
            # res = detect_human_to_tf()
            voice.talk('I will check if this place is empty')
            # res , _ = wait_for_face()  # seconds
            res = 'no_face'
            if res == 'no_face':

                print("Place is: ",place)
                face = place.replace('_', '_face')
                guest = party.get_active_guest_name()
                
                # Get angle from robot to place and turn base
                traslation, rotation = tf_manager.get_transform(target_frame = face,
                                        source_frame = 'base_link')
                
                
                _, _, theta = euler_from_quaternion(rotation)
                
                
                angle = np.arctan2(traslation[1], traslation[0])

                angle -= theta + 3.14
                head.request_base_turn(angle)

                head.set_named_target('neutral')
                rospy.sleep(0.8)

                # brazo.set_named_target('neutral')
                arm.set_named_target('point')
                voice.talk(f'{guest}, I found you a free place, sit here please.')


                party.seat_confirmation()
                self.tries = 0 
                return 'succ'

            else:
                # There could be 2 "someones"
                # TODO: change this to implement 2 "someones" instead of 1
                # occupant_name = res.Ids.ids
                occupant_name = None
                if occupant_name == 'unknown':
                    occupant_name = 'someone'
                party.seat_confirmation(occupant_name)
                voice.talk(f'I am sorry, here is {occupant_name}, I will find another place for you.')
                return 'failed'
        except rospy.ROSInterruptException:
            return 'aborted'

class Check_party(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['failed', 'guest_done', 'party_done', 'aborted'])
        self.tries = 0
    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo("State: Check party status")
        arm.set_named_target('go')

        try:
            if party.guest_assigned == 1:
                return 'guest_done'
            elif party.guest_assigned == 2:
                return 'party_done'
            else:
                return 'failed'
        except rospy.ROSInterruptException:
            return 'aborted'
# Find guest STATE: Look for both guests to introduce each other

class Find_guests(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['succ', 'failed', 'aborted'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo("STATE: Find guests to introduces each other")

        try:
            # First try: find guest_0
            # Second try: find guest_1

            guest_name = party.people[f'Guest_{self.tries}']['name']
            guest_loc = party.people[f'Guest_{self.tries}']['location']

            voice.talk(f'Looking for {guest_name} on: {guest_loc}')
            tf_host = guest_loc.replace('_', '_face')
            head.look_at_frame(tf_host)

            rospy.sleep(0.7)
            self.tries += 1
            return 'succ'

        except rospy.ROSInterruptException:
            return 'aborted'

# Introduce guest STATE: Introduce each other guests
        
class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'next', 'aborted'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Introduce guests')
        try:
            self.tries += 1
            if self.tries == 1:
                gtt = 'Guest_0'
                gti = 'Guest_1'
            elif self.tries == 2:
                gtt = 'Guest_1'
                gti = 'Guest_0'
                self.tries = 0

            guest_to_talk = party.people[gtt]
            guest_to_introduce = party.people[gti]
            print(f'Guest to talk: {guest_to_talk}')
            print(f'Guest to introduce: {guest_to_introduce}')

            # Introduce guests
            gtt_name = guest_to_talk['name']
            gti_name = guest_to_introduce['name']
            voice.talk(f'{gtt_name}, I would like to introduce you to {gti_name}')
            speech = ''

            # Description line
            if guest_to_introduce['description']:
                description = guest_to_introduce['description']
                speech += f'{description}, '

            # Drink line
            if guest_to_introduce['drink'] != 'something':
                drink = guest_to_introduce['drink']
                speech += f'likes to drink {drink}, '
            
            # Interest line
            if guest_to_introduce['interest']:
                interest = guest_to_introduce['interest']
                speech += f'is interested in {interest}'
            
            voice.talk(speech)
            
            if self.tries == 1:
                return 'next'
            else:
                rospy.sleep(1.0)
                voice.talk("Task completed, thanks for watching")
                return 'succ'

        except rospy.ROSInterruptException:
            return 'aborted'
        
# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['success', 'failure'])

    with sm:
        # State machine for Receptionist task
        if debug:
            # omni_base.set_debug(debug)
            voice.talk('Debug mode')

        # Initial states routine
        smach.StateMachine.add("INITIAL", Initial(), 
                               transitions = {'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND', 'aborted': 'failure'})
        
        smach.StateMachine.add("WAIT_PUSH_HAND", 
                               WaitPushHand(
                                   talker = voice, 
                                   talk_message = 'Please, push my hand to begin',
                                   timeout = 100.0,
                                   push_threshold = 0.7),
                                transitions = {'failed': 'WAIT_PUSH_HAND', 'succ': 'GOTO_DOOR', 'aborted': 'failure'})  
        
        smach.StateMachine.add("WAIT_DOOR_OPENED", 
                               WaitDoorOpen(
                                   talker = voice,
                                   talk_message = 'Open the door please', 
                                   timeout = 100.0,
                                   distance_threshold = 0.5),
                                transitions = {'failed': 'WAIT_DOOR_OPENED', 'succ': 'SCAN_FACE', 'aborted': 'failure'})
        
        # Guest recognition states
        smach.StateMachine.add("SCAN_FACE", Scan_face(),
                               transitions = {'failed': 'SCAN_FACE', 'succ': 'DECIDE_FACE', 'aborted': 'failure'})
        
        smach.StateMachine.add("DECIDE_FACE", Decide_face(),
                               transitions = {'failed': 'SCAN_FACE', 'succ': 'GET_DRINK', 'unknown': 'NEW_FACE', 'aborted': 'failure'})
        
        smach.StateMachine.add("NEW_FACE", New_face(),     
                               transitions = {'failed': 'NEW_FACE', 'succ': 'GET_DRINK', 'aborted': 'failure'})
        
        smach.StateMachine.add("GET_DRINK", Get_drink(),    
                               transitions = {'failed': 'GET_DRINK', 'succ': 'GET_INTEREST', 'aborted': 'failure'})
        
        smach.StateMachine.add("GET_INTEREST", Get_interest(),    
                               transitions = {'failed': 'GET_INTEREST', 'succ': 'LEAD_TO_BEVERAGE_AREA', 'aborted': 'failure'})

        # Guest treatment
        smach.StateMachine.add("LEAD_TO_BEVERAGE_AREA", 
                               GotoPlace(
                                   navigation = omni_base, 
                                   location = 'beverage_area',
                                   talker = voice,
                                   start_message = 'I will take you to beverage area, please follow me', 
                                   debug = debug),
                               transitions = {'failed': 'LEAD_TO_BEVERAGE_AREA', 'succ': 'FIND_DRINK', 'aborted': 'failure'})
        
        smach.StateMachine.add("FIND_DRINK", Find_drink(),
                               transitions = {'failed': 'FIND_DRINK', 'succ': 'LEAD_TO_LIVING_ROOM', 'aborted': 'failure'})
        
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM", 
                               GotoPlace(
                                   navigation = omni_base, 
                                   location = 'living_room', 
                                   talker = voice, 
                                   start_message = "Follow me to living room",
                                   debug = debug),
                                transitions = {'failed': 'LEAD_TO_LIVING_ROOM', 'succ': 'FIND_SITTING_PLACE', 'aborted': 'failure'})
        
        smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(),
                               transitions = {'failed': 'FIND_SITTING_PLACE', 'succ': 'CHECK_PARTY', 'aborted': 'failure'})
        
        smach.StateMachine.add("CHECK_PARTY", Check_party(),
                               transitions = {'failed': 'CHECK_PARTY', 'guest_done': 'GOTO_DOOR', 'party_done': 'FIND_GUEST', 'aborted': 'failure'})
        
        smach.StateMachine.add("GOTO_DOOR", 
                               GotoPlace(
                                   navigation = omni_base, 
                                   location = 'door', 
                                   talker = voice, 
                                   start_message = 'Navigating to door',
                                   debug = debug),
                                transitions = {'failed': 'GOTO_DOOR', 'succ': 'WAIT_DOOR_OPENED', 'aborted': 'failure'})
        
        # Introducing guests
        smach.StateMachine.add("FIND_GUEST", Find_guests(),
                               transitions = {'failed': 'FIND_GUEST', 'succ':'INTRODUCE_GUEST', 'aborted': 'failure'})
        
        smach.StateMachine.add("INTRODUCE_GUEST", Introduce_guest(),
                               transitions = {'next': 'FIND_GUEST', 'succ':'success', 'aborted': 'failure'})
    
    # Execute the state machine
    rospy.loginfo("Executing state machine...")
    outcome = sm.execute()
