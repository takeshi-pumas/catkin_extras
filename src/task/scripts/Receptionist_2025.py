#!/usr/bin/env python3
from smach_utils_receptionist import *


# Initial STATE: task setup (grammar, knowledge, robot position, ...)

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo('STATE : INITIAL')
        rospy.loginfo(f'Try {self.tries} of 5 attempts')

        party.clean_knowledge(host_name = "john", host_location = "Place_3")
        #party.clean_knowledge()
        #places_2_tf()
        #party.publish_tf_seats()
        places_2_tf()

        ###-----INIT GRAMMAR FOR VOSK
        ###-----Use with get_keywords_speech()
        ###-----------SPEECH REC
        #drinks=['coke','juice','beer', 'water', 'soda', 'wine', 'i want a', 'i would like a']
        #drinks = ['coke','juice','milk', 'water', 'soda', 'wine', 
        #          'i want a', 'i would like a', 'tea', 'icedtea', 'cola', 'redwine', 'orangejuice', 'tropicaljuice']
        drinks = ['water', 'soda', 'coke', 'juice', 'iced tea', 'i want a', 'i would like a']
        #names=['rebeca','ana','jack', 'michael', ' my name is' , 'i am','george','mary','ruben','oscar','yolo','mitzi']
        #names = [' my name is' , 'i am','adel', 'angel', 'axel', 
        #         'charlie', 'jane', 'john', 'jules', 'morgan', 'paris', 'robin', 'simone', 'jack']
        names = ['my name is', 'i am','john', 'jack', 'paris', 'charlie', 'simone', 'robin', 'jane', 'jules']
        confirmation = ['yes','no', 'robot yes', 'robot no','not','now','nope','yeah']                     
        gram = drinks + names + confirmation                                                                                
        
        if self.tries == 1:
            set_grammar(gram)  ##PRESET DRINKS
            rospy.sleep(0.2)
            return 'succ'
        elif self.tries == 3:
            return 'failed'

# Wait push hand STATE: Trigger for task to start

class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Wait for Wait_push_hand')
        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        # if self.tries == 4:
        #     return 'failed'
        head.set_named_target('neutral')
        brazo.set_named_target('go')
        voice.talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'

# Wait door opened STATE: Trigger for task to start

class Wait_door_opened(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for door to be opened')
        print('Waiting for door to be opened')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')

        # if self.tries == 100:
        #     return 'tries'
        voice.talk('I am ready for receptionist task.')
        rospy.sleep(0.8)
        voice.talk('I am waiting for the door to be opened')
        succ = line_detector.line_found()
        #succ = wait_for_push_hand(100)
        rospy.sleep(1.0)
        if succ:
            self.tries = 0
            return 'succ'
        else:
            return 'failed'

# Go to door STATE: Move robot to known location "door" 

class Goto_door(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location: Door')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'succ'
        if self.tries == 1: voice.talk('Navigating to, door')
        res = omni_base.move_base(known_location = 'door')
        print(res)

        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
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
        head.set_joint_values([0.0, 0.3])
        voice.talk('Waiting for guest, look at me, please')
        res, userdata.face_img = wait_for_face()  # default 10 secs
        #rospy.sleep(0.7)
        if res != None:
            userdata.name = res.Ids.ids
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
            confirmation = get_keywords_speech(10)
            print (confirmation)

            if confirmation not in ['yes','jack','juice', 'takeshi yes','yeah']:
                return 'unknown'
            elif confirmation == "timeout":
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
        self.tries += 1
        rospy.loginfo('STATE : NEW_FACE')
        #If name is not recognized 3 times, guest will be registered as a "someone"
        if self.tries == 3:
            voice.talk ('I didnt undestand your name, lets continue')
            userdata.name = 'someone'
            train_face(userdata.face_img, userdata.name )
            party.add_guest(userdata.name)
            self.tries = 0
            return 'succ'
        
        #Asking for name
        voice.talk('Please, tell me your name')
        #rospy.sleep(1.0)
        speech = get_keywords_speech(10)
        # in case thinks like I am , my name is . etc
        if len(speech.split(' ')) > 1: name = (speech.split(' ')[-1])
        else: name = speech

        if userdata.name == 'timeout':
            voice.talk('Please repeat it and speak louder.')
            return 'failed'

        voice.talk(f'Is {name} your name?')
        #rospy.sleep(2.0)
        confirmation = get_keywords_speech(10)
        print (confirmation)

        confirmation = confirmation.split(' ')
        confirm = match_speech(confirmation, ['yes','yeah','jack','juice'])
        if confirm:
            userdata.name = name
            voice.talk (f'Nice to Meet You {userdata.name}')
            party.add_guest(userdata.name)
            train_face(userdata.face_img, userdata.name)
            self.tries = 0
            return 'succ'
        else:
            voice.talk ('lets try again')
            return 'failed'

# Get drink STATE: Ask guest for their favourite drink

class Get_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succ', 'failed'],
                             input_keys=['name', 'face_img'])
        #self.new_name = ''
        #self.num_faces = 0
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo('STATE : GET DRINK')

        if self.tries == 1:
            analyze_face_background(userdata.face_img, party.get_active_guest_name())#userdata.name)
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
        drink = get_keywords_speech(10)

        if len(drink.split(' '))>1: drink=(drink.split(' ')[-1])
        print(drink)
        rospy.sleep(0.5)

        if drink=='timeout':
            voice.talk("Sorry, couldn't hear you. Please speak louder.")
            return 'failed' 
        voice.talk(f'Did you say {drink}?')

        #rospy.sleep(2.5)
        confirmation = get_keywords_speech(10)
        confirmation = confirmation.split(' ')
        confirm = match_speech(confirmation, ['yes','yeah','jack','juice'])
        if not confirm: return 'failed' 

        party.add_guest_drink(drink)
        voice.talk("Nice")
        self.tries = 0
        return 'succ'

# Get interest STATE: Ask guest for their interest

class Get_interest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcome=['succ', 'failed'])
        self.tries = 0
    
    def execute(self, userdata):
        rospy.loginfo('STATE : Ask guest for any interest')
        print('Try', self.tries, 'of 3 attempts')

        voice.talk('which interest do you have?')
        
        response = get_keywords_speech(10)
        
        #TODO: save interest response to the yaml

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
        res = omni_base.move_base(known_location='beverage_area')
        if res:
            self.tries = 0
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
        print('Try', self.tries, 'of 3 attempts')

        voice.talk('Scanning table')
        res = False

        #TODO: recognize drinks on table and give a match (if available) position (left, center, right)
        position = 'center'

        if res:
            self.tries = 0

            voice.talk(f"I found a {party.get_active_guest_drink()} on the {position}, take it please.")
            return 'succ'
        else:
            voice.talk(f'There is no {party.get_active_guest_drink()}, if you want to, take another one.')
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
        res = omni_base.move_base(known_location='living_room')
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
        isPlace, place = party.get_active_seat()
        # isLocation, loc = party.get_active_seat_location()
        # if isLocation:
        #     #omni_base.move_base(*loc)
        #     pass
        # else:
        #     voice.talk('Sorry, there is no more places to sit')
        #     return 'succ'

        if isPlace:
            tf_name = place.replace('_', '_face')
            head.to_tf(tf_name)

        #commented detect human
        #res = detect_human_to_tf()
        voice.talk('I will check if this place is empty')
        res , _ = wait_for_face()  # seconds
        if not res:

            print("Place is: ",place)
            guest = party.get_active_guest_name()
            head.turn_base_gaze2(tf = place, to_gaze = 'base_link')
            head.set_named_target('neutral')
            rospy.sleep(0.8)

            brazo.set_named_target('neutral')
            voice.talk(f'{guest}, I found you a free place, sit here please.')


            party.seat_confirmation()
            self.tries = 0 
            return 'succ'

        else:
            # There could be 2 "someones"
            #TODO: change this to implement 2 "someones" instead of 1
            occupant_name = res.Ids.ids
            if occupant_name == 'unknown':
                occupant_name = 'someone'
            party.seat_confirmation(occupant_name)
            voice.talk(f'I am sorry, here is {occupant_name}, I will find another place for you.')
            return 'failed'

# Find guest STATE: Look for both guests to introduce each other

class Find_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['succ', 'failed'],
                             output_keys=['name_like_host'])
        self.tries = 0
    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo("STATE: Find guest alternative")
        
        host_name = ""
        host_loc = ""
        dont_compare = False

        # First try: looks for the real host
        # Next tries: looks for anyone

        if self.tries == 1:
            host_name, host_loc = party.get_host_info()
            if host_loc == 'None':
                return 'failed'
        else:
            places = party.get_places()
            isActive, seat = party.get_active_seat()
            places.remove('Place_0')
            if isActive:
                places.remove(seat)

            print('#####', places)

            host_loc = places[self.tries - 2] #places[0]

            
            if self.tries - 1 >= len(places):
                self.tries = 1 
            #host_loc = seat
            dont_compare = True
        
        #print("host location is: ", host_loc)
        #print("host name is: ", host_name)
        voice.talk(f'Looking for host on: {host_loc}')
        tf_host = host_loc.replace('_', '_face')
        head.to_tf(tf_host)
        voice.talk("look at me, please")
        rospy.sleep(0.7)
        res, _ = wait_for_face()
        if res is not None:
            person_name = res.Ids.ids
            if (person_name == host_name) or dont_compare:
                userdata.name_like_host = person_name
                return 'succ'
            else:
                return 'failed'
        else:
            return 'failed'

# Introduce guest STATE: Introduce each other guests
        
class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],
                             input_keys=['name_like_host'])
        self.tries = 0

    def execute(self, userdata):
        #global name_like_host
        rospy.loginfo('STATE : Find host')

        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')

        voice.talk(f'Host like name is {userdata.name_like_host}')

        active_guest = party.get_active_guest_name()
        takeshi_line = party.get_active_guest_description()                    
        drink = party.get_active_guest_drink()

        if drink == 'something':
            drink_line = ""
        else:
            drink_line = f'And likes {drink}'

        if takeshi_line != 'None':
            print("Description found")
            speech = f'{userdata.name_like_host}, {takeshi_line}, {drink_line}'
            timeout = 14.0
        else:
            print('No description found')
            speech = f'{userdata.name_like_host}, {active_guest} has arrived, {drink_line}'
            timeout = 7.0

        voice.talk(speech, timeout)
        
        if self.tries < 3:
            return 'succ'
        else:
            voice.talk("Task completed, thanks for watching")
            return 'tries'

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])

    # sm.userdata.clear = False
    #sis = smach_ros.IntrospectionServer(
    #    'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    #sis.start()

    with sm:
        # State machine for Receptionist task

        # Initial states routine
        smach.StateMachine.add("INITIAL", Initial(),              
                               transitions={'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND'})
        smach.StateMachine.add("WAIT_PUSH_HAND", Wait_push_hand(),       
                               transitions={'failed': 'WAIT_PUSH_HAND', 'succ': 'GOTO_DOOR'})
        smach.StateMachine.add("WAIT_DOOR_OPENED", Wait_door_opened(),     
                               transitions={'failed': 'WAIT_DOOR_OPENED', 'succ': 'GOTO_DOOR'})
        smach.StateMachine.add("GOTO_DOOR", Goto_door(),            
                               transitions={'failed': 'GOTO_DOOR', 'succ': 'SCAN_FACE'})
        
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
        smach.StateMachine.add("LEAD_TO_BEVERAGE_AREA", Lead_to_beverage_area(),  
                               transitions={'failed': 'LEAD_TO_BEVERAGE_AREA', 'succ': 'FIND_SITTING_PLACE'})
        smach.StateMachine.add("FIND_DRINK", Find_drink(),
                               transitions={'failed': 'FIND_DRINK', 'succ': 'LEAD_TO_LIVING_ROOM'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM", Lead_to_living_room(),  
                               transitions={'failed': 'LEAD_TO_LIVING_ROOM', 'succ': 'FIND_SITTING_PLACE'})
        smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(),
                               transitions={'failed': 'FIND_SITTING_PLACE', 'succ': 'FIND_HOST'})
        
        # Introducing guests
        smach.StateMachine.add("FIND_HOST", Find_host_alternative(),
                               transitions={'failed': 'FIND_HOST', 'succ':'INTRODUCE_GUEST'})
        smach.StateMachine.add("INTRODUCE_GUEST", Introduce_guest(),
                               transitions={'failed': 'INTRODUCE_GUEST', 'succ':'WAIT_PUSH_HAND', 'tries':'END'})


    outcome = sm.execute()