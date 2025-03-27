#!/usr/bin/env python3
import rospy
import smach
import numpy as np
# from smach_utils_receptionist import *
from common.states import WaitPushHand, WaitDoorOpen, GotoPlace
from common.navigation_functions import Navigation
from common.hsr_functions import Talker, Gaze, ArmController
from common.ros_functions import TFManager
from common.logic import Receptionist

confirmation = ('yes', 'robot yes','yeah', 
                'correct', 'right', 'that is right', 
                'that is correct', 'that is it')

# Initialize objects
voice = Talker()
tf_manager = TFManager()
omni_base = Navigation()
party = Receptionist()
head = Gaze()
arm = ArmController(omni_base, tf_manager)

def publish_places():
    """Publish known locations to TF"""
    places, locations = party.get_places_location()
    for i in range(len(places)):
        tf_manager.publish_transform(
            position=[locations[i][0], locations[i][1], 0.0],
            rotation=[0.0, 0.0, 0.0],
            child_frame=places[i],
            parent_frame="map",
            static=True
        )
    rospy.sleep(0.7)


# Initial STATE: task setup (grammar, knowledge, robot position, ...)

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        global confirmation
        self.tries += 1
        rospy.loginfo('STATE : INITIAL')
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
        declination = ('no','nope','not','now','robot no', 'incorrect', 'wrong', 'that is wrong', 'that is not right', 'that is not correct')       
        gram = list(drinks + names + confirmation + declination)                                                                       
        
        if self.tries == 1:
            # set_grammar(gram)  ##PRESET DRINKS
            rospy.sleep(0.2)
            return 'succ'
        elif self.tries == 3:
            return 'failed'

# Scan face STATE: Take a picture of the new guest to meet them

class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ['succ', 'failed'], 
                             output_keys = ['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('State : Scan face')
        head.move_head((0.0, 0.3))
        voice.talk('Waiting for guest, look at me, please')
        # res, userdata.face_img = wait_for_face()  # default 10 secs
        #rospy.sleep(0.7)

        res = 'Ruben'
        if res != None:
            # userdata.name = res.Ids.ids
            userdata.name = res
            return 'succ'
        else:
            return 'failed'

# Decide face STATE: Decide if the new guest is known, unknown or can't be decided

class Decide_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed', 'unknown'], 
                             input_keys=['name', 'face_img'], 
                             output_keys=['name', 'face_img'])
        self.tries = 0
    def execute(self, userdata):
        global confirmation
        self.tries += 1
        rospy.loginfo("STATE: Decide face")

        if userdata.name == 'NO_FACE':
            voice.talk('I did not see you, I will try again')
            return 'failed'

        elif userdata.name == 'unknown':
            voice.talk('I believe we have not met.')
            self.tries = 0
            return 'unknown'

        else:
            voice.talk(f'I found you, I Think you are, {userdata.name}.')
            voice.talk('Is it correct?')
            #rospy.sleep(2.5)
            # speech = get_keywords_speech(10)
            speech = 'yes'
            print (speech)

            if speech not in confirmation:
                return 'unknown'
            elif speech == "timeout":
                voice.talk('I could not hear you, lets try again, please speak louder.')
                return "failed"
            else:
                self.tries = 0
                party.add_guest(userdata.name)
                return 'succ'

# New face STATE: Train new guest on DB

class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed'],
                             input_keys=['name', 'face_img'],
                             output_keys=['name', 'face_img'])
        #self.new_name = ''
        #self.num_faces = 0
        self.tries = 0

    def execute(self, userdata):
        global confirmation
        self.tries += 1
        rospy.loginfo('STATE : NEW_FACE')
        #If name is not recognized 3 times, guest will be registered as a "someone"
        if self.tries == 3:
            voice.talk ('I didnt undestand your name, lets continue')
            userdata.name = 'someone'
            # train_face(userdata.face_img, userdata.name )
            party.add_guest(userdata.name)
            self.tries = 0
            return 'succ'
        
        #Asking for name
        voice.talk('Please, tell me your name')
        #rospy.sleep(1.0)
        # speech = get_keywords_speech(10)
        speech = 'john'
        # in case thinks like I am , my name is . etc
        if len(speech.split(' ')) > 1: name = (speech.split(' ')[-1])
        else: name = speech

        if userdata.name == 'timeout':
            voice.talk('Please repeat it and speak louder.')
            return 'failed'

        voice.talk(f'Is {name} your name?')
        #rospy.sleep(2.0)
        # speech = get_keywords_speech(10)
        speech = 'yes'
        print (speech)

        speech = speech.split(' ')
        # confirm = match_speech(speech, ('yes','yeah','jack','juice'))

        if speech not in confirmation:
            voice.talk ('lets try again')
            return 'failed'
        
        userdata.name = name
        voice.talk (f'Nice to Meet You {userdata.name}')
        party.add_guest(userdata.name)
        # train_face(userdata.face_img, userdata.name)
        self.tries = 0
        return 'succ'


# Get drink STATE: Ask guest for their favourite drink

class Get_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed'],
                             input_keys=['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        global confirmation
        self.tries += 1
        rospy.loginfo('STATE : GET DRINK')

        if self.tries == 1:
            # analyze_face_background(userdata.face_img, userdata.name)
            print(userdata.face_img.shape)

        elif self.tries == 3:
            voice.talk ('I am having trouble understanding you, lets keep going')
            drink = 'something'
            self.tries=0
            party.add_guest_drink(drink)
            #analyze_face_background(userdata.face_img, userdata.name)
            return 'succ'
        #Asking for drink
        voice.talk('What would you like to drink?')
        #rospy.sleep(2.0)
        # drink = get_keywords_speech(10)
        drink = 'juice'

        if len(drink.split(' '))>1: drink=(drink.split(' ')[-1])
        print(drink)
        rospy.sleep(0.5)

        if drink=='timeout':
            voice.talk("Sorry, couldn't hear you. Please speak louder.")
            return 'failed' 
        voice.talk(f'Did you say {drink}?')

        #rospy.sleep(2.5)
        # speech = get_keywords_speech(10)
        speech = 'yes'
        speech = speech.split(' ')
        # confirm = match_speech(speech, ('yes','yeah','jack','juice'))
        if speech not in confirmation: 
            return 'failed' 

        party.add_guest_drink(drink)
        voice.talk("Nice")
        self.tries = 0
        return 'succ'

# Get interest STATE: Ask guest for their interest

class Get_interest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    
    def execute(self, userdata):
        rospy.loginfo('STATE : Ask guest for any interest')
        print('Try', self.tries, 'of 3 attempts')

        voice.talk('which interest do you have?')
        
        # response = get_keywords_speech(10)
        response = 'music'
        party.add_guest_interest(response)

        voice.talk("Nice")
        self.tries = 0
        return 'succ'

# Lead to beverage area STATE: Ask guest to follow robot to beverage area

class Lead_to_beverage_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : navigate to known beverage area')
        print('Try', self.tries, 'of 3 attempts')

        voice.talk(f'{party.get_active_guest_name()}... I will take you to beverage area, please follow me')
        # voice.talk('Navigating to ,living room')
        res = omni_base.move_to(known_location='beverage_area')
        if res:
            self.tries = 0
            voice.talk(f"I will check if there is {party.get_active_guest_drink()} here")
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'

# Find drink STATE: Scan beverage area and detect where is guest's favourite drink

class Find_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Scan table to find beverages')
        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')

        voice.talk('Scanning table')
        head.move_head((0.0, -0.3))
        rospy.sleep(1)

        favorite_drink = party.get_active_guest_drink()
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

# Lead to living room STATE: Ask guest to follow robot to living room

class Lead_to_living_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : navigate to known location')
        print('Try', self.tries, 'of 3 attempts')

        voice.talk(f"{party.get_active_guest_name()}... Follow me to living room")
        # voice.talk('Navigating to ,living room')
        res = omni_base.move_to(known_location='living_room')
        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'

# Find sitting place STATE: Find a place to sit according to previous knowledge of the world

class Find_sitting_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):

        rospy.loginfo('STATE : Looking for a place to sit')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        voice.talk('I am looking for a place to sit')
        place = party.get_active_seat()

        if place:
            tf_name = place.replace('_', '_face')
            head.look_at_frame(tf_name)

        # commented detect human
        # res = detect_human_to_tf()
        voice.talk('I will check if this place is empty')
        # res , _ = wait_for_face()  # seconds
        res = 'no_face'
        if not res:

            print("Place is: ",place)
            guest = party.get_active_guest_name()
            
            # Get angle from robot to place and turn base
            traslation, _ = tf_manager.get_transform(target_frame = place,
                                     source_frame = 'base_link')
            
            angle = np.arctan2(traslation[1], traslation[0])
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

class Check_party(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['failed', 'guest_done', 'party_done'])
        self.tries = 0
    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo("State: Check party status")

        if party.guest_assigned == 1:
            return 'guest_done'
        elif party.guest_assigned == 2:
            return 'party_done'
        else:
            return 'failed'



# Find guest STATE: Look for both guests to introduce each other

class Find_guests(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['succ', 'failed'],
                             output_keys=['name_like_host'])
        self.tries = 0
    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo("STATE: Find guests to introduces each other")

        # First try: find first guest
        # Second try: find second guest

        guest_name = party.people[f'Guest_{self.tries}'].name
        guest_loc = party.people[f'Guest_{self.tries}'].location

        voice.talk(f'Looking for {guest_name} on: {guest_loc}')
        tf_host = guest_loc.replace('_', '_face')
        head.look_at_frame(tf_host)

        rospy.sleep(0.7)
        return 'succ'

# Introduce guest STATE: Introduce each other guests
        
class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'next'],
                             input_keys=['name_like_host'])
        self.tries = 0

    def execute(self, userdata):
        #global name_like_host
        rospy.loginfo('STATE : Find host')

        self.tries += 1
        if self.tries == 1:
            gtt = 'Guest_1'
            gti = 'Guest_2'
        elif self.tries == 2:
            gtt = 'Guest_2'
            gti = 'Guest_1'
            self.tries = 0

        guest_to_talk = party.people[gtt]
        guest_to_introduce = party.people[gti]

        # Introduce guests

        voice.talk(f'{guest_to_talk.name}, I would like to introduce you to {guest_to_introduce.name}')
        speech = ''
        timeout = 0.0

        # Description line
        if guest_to_introduce.description:
            speech = guest_to_introduce.description
            timeout += 10.0

        # Drink line
        if guest_to_introduce.drink != 'something':
            speech += f'likes to drink {guest_to_introduce.drink}'
            timeout += 5.0
        
        # Interest line
        if guest_to_introduce.interest:
            speech = f'and is interested in {guest_to_introduce.interest}'
            timeout += 5.0
        
        voice.talk(speech, timeout)
        
        if self.tries == 0:
            return 'next'
        else:
            rospy.sleep(3.0)
            voice.talk("Task completed, thanks for watching")
            return 'succ'

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])

    with sm:
        # State machine for Receptionist task

        # Initial states routine
        smach.StateMachine.add("INITIAL", Initial(),              
            transitions={'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND'})
        smach.StateMachine.add("WAIT_PUSH_HAND", WaitPushHand(
            talker=voice, talk_message='Gently... push my hand to begin', timeout=100.0, push_threshold=0.3),       
            transitions={'failed': 'WAIT_PUSH_HAND', 'succ': 'GOTO_DOOR'})
        smach.StateMachine.add("WAIT_DOOR_OPENED", WaitDoorOpen(
            talker=voice, talk_message='Open the door please', timeout=100.0, distance_threshold=0.5),     
            transitions={'failed': 'WAIT_DOOR_OPENED', 'succ': 'SCAN_FACE'})
        
        # Guest recognition states
        smach.StateMachine.add("SCAN_FACE", Scan_face(),
                               transitions={'failed': 'SCAN_FACE', 'succ': 'DECIDE_FACE'})
        smach.StateMachine.add("DECIDE_FACE", Decide_face(),
                               transitions={'failed': 'SCAN_FACE', 'succ': 'GET_DRINK', 'unknown': 'NEW_FACE'})
        smach.StateMachine.add("NEW_FACE", New_face(),     
                               transitions={'failed': 'NEW_FACE', 'succ': 'GET_DRINK'})
        smach.StateMachine.add("GET_DRINK", Get_drink(),    
                               transitions={'failed': 'GET_DRINK', 'succ': 'GET_INTEREST'})
        smach.StateMachine.add("GET_INTEREST", Get_interest(),    
                               transitions={'failed': 'GET_INTEREST', 'succ': 'LEAD_TO_BEVERAGE_AREA'})

        # Guest treatment
        smach.StateMachine.add("LEAD_TO_BEVERAGE_AREA", GotoPlace(
            navigation=omni_base, location='beverage_area', talker=voice, start_message=f'{party.get_active_guest_name()}... I will take you to beverage area, please follow me', 
            end_message=f"I will check if there is {party.get_active_guest_drink()} here"),
                               transitions={'failed': 'LEAD_TO_BEVERAGE_AREA', 'succ': 'FIND_DRINK'})
        smach.StateMachine.add("FIND_DRINK", Find_drink(),
                               transitions={'failed': 'FIND_DRINK', 'succ': 'LEAD_TO_LIVING_ROOM'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM", GotoPlace(
            navigation=omni_base, location='living_room', talker=voice, start_message=f"{party.get_active_guest_name()}... Follow me to living room"),  
            transitions={'failed': 'LEAD_TO_LIVING_ROOM', 'succ': 'FIND_SITTING_PLACE'})
        smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(),
                               transitions={'failed': 'FIND_SITTING_PLACE', 'succ': 'CHECK_PARTY'})
        smach.StateMachine.add("CHECK_PARTY", Check_party(),
                               transitions={'failed': 'CHECK_PARTY', 'guest_done': 'GOTO_DOOR', 'party_done': 'FIND_GUEST'})
        smach.StateMachine.add("GOTO_DOOR", GotoPlace(
            navigation=omni_base, location='door', talker=voice, start_message='Navigating to door', end_message='Arrived to door'),
            transitions={'failed': 'GOTO_DOOR', 'succ': 'WAIT_DOOR_OPENED'})
        
        # Introducing guests
        smach.StateMachine.add("FIND_GUEST", Find_guests(),
                               transitions={'failed': 'FIND_GUEST', 'succ':'INTRODUCE_GUEST'})
        smach.StateMachine.add("INTRODUCE_GUEST", Introduce_guest(),
                               transitions={'next': 'FIND_GUEST', 'succ':'END'})


    outcome = sm.execute()