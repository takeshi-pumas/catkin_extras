#!/usr/bin/env python3

from smach_utils_receptionist import *

##### Define state INITIAL #####

# --------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')

            
        clean_knowledge()
        places_2_tf()
        ###-----INIT GRAMMAR FOR VOSK
        ###-----Use with get_keywords_speech()
        ###-----------SPEECH REC
        drinks=['coke','juice','beer', 'water', 'soda', 'wine', 'i want a', 'i would like a']
        names=['rebeca','ana','jack', 'michael', ' my name is' , 'i am','george','mary','ruben','oscar','yolo','mitzi']
        confirmation=['yes','no', 'takeshi yes', 'takeshi no','not','now','nope','yeah']                                                                                          
        gram=drinks+names+confirmation                                                                                
        if self.tries == 1:
            set_grammar(gram)  ##PRESET DRINKS
        elif self.tries == 3:
            return 'tries'

        

        rospy.sleep(0.8)
        return 'succ'

# --------------------------------------------------

class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for Wait_push_hand')
        print('Waiting for hand to be pushed')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            return 'tries'
        head.set_named_target('neutral')
        brazo.set_named_target('go')
        voice.talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'

# --------------------------------------------------

class Wait_door_opened(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for door to be opened')
        print('Waiting for door to be opened')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')

        if self.tries == 100:
            return 'tries'
        voice.talk('I am ready for receptionist task.')
        rospy.sleep(0.8)
        voice.talk('I am waiting for the door to be opened')
        succ = line_detector.line_found()
        #succ = wait_for_push_hand(100)
        rospy.sleep(1.0)
        if succ:
            return 'succ'
        else:
            return 'failed'

# --------------------------------------------------

class Goto_door(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location: Door')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: voice.talk('Navigating to, door')
        res = omni_base.move_base(known_location = 'door')
        print(res)

        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'

# --------------------------------------------------

class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'unknown', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global  img_face, name_face

        rospy.loginfo('State : Scan face')
        head.set_joint_values([0.0, 0.3])
        voice.talk('Scanning for faces, look at me, please')
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        name_face = ""
        res, img_face = wait_for_face()  # default 10 secs
        rospy.sleep(0.7)
        print (res)

        print('Checking for faces')
        if res != None:
            name = res.Ids.ids

            print('RESPONSE', name)
            if name == 'NO_FACE':
                print('No face Found, Keep scanning')
                voice.talk('I did not see you, I will try again')
                return 'failed'

            elif name == 'unknown':
                print('A face was found.')
                voice.talk('I believe we have not met. ')
                return 'unknown'

            else:
                voice.talk(f'I found you, I Think you are, {name}.')
                voice.talk('Is it correct?')
                rospy.sleep(2.5)
                confirmation = get_keywords_speech(10)
                print (confirmation)

                if confirmation not in ['yes','jack','juice', 'takeshi yes','yeah']:
                    return 'unknown'
                elif confirmation == "timeout":
                    voice.talk('I could not hear you')
                    return "failed"
                else:
                    name_face = name
                    return 'succ'
        else:
            return 'failed'

# --------------------------------------------------

class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.new_name = ''
        self.num_faces = 0
        self.tries = 0

    def execute(self, userdata):
        global name_face
        self.tries += 1

        rospy.loginfo('STATE : NEW_FACE')
        
        #If name is not recognized 3 times, guest may be registered as a "someone"
        if self.tries == 4:
            voice.talk ('I am having trouble understanding your name, lets keep going')
            name = 'someone'
            name_face=name
            train_face(img_face, name)
            self.tries=0
            return 'succ'
        
        #Asking for name
        voice.talk('Please, tell me your name')
        rospy.sleep(1.0)
        speech=get_keywords_speech(10)
        # in case thinks like I am , my name is . etc
        if len(speech.split(' '))>1: name=(speech.split(' ')[-1])
        else: name = speech
        
        name_face=name
        print (name)

        if  name=='timeout':
            voice.talk('I could not hear you , lets try again')
            #voice.talk ('lets try again')
            return 'failed'

        voice.talk(f'Is {name} your name?')
        rospy.sleep(2.0)
        confirmation = get_keywords_speech(10)
        print (confirmation)
        
        if confirmation in['yes','jack','juice','yeah']:   ### YES AND MOST COMMON MISREADS
            voice.talk (f'Nice to Meet You {name}')
            train_face(img_face, name)
            self.tries=0
            return 'succ'
        else:
            voice.talk ('lets try again')
            return 'failed'

# --------------------------------------------------

class Get_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.new_name = ''
        self.num_faces = 0
        self.tries = 0

    def execute(self, userdata):
        global name_face , drink
        self.tries += 1

        rospy.loginfo('STATE : GET DRINK')

        #Asking for drink
        voice.talk('What would you like to drink?')
        rospy.sleep(2.0)
        drink=get_keywords_speech(10)

        if len(drink.split(' '))>1: drink=(drink.split(' ')[-1])

        rospy.sleep(0.5)
        voice.talk(f'Did you say {drink}?')

        rospy.sleep(4.0)
        confirmation = get_keywords_speech(10)
        print (confirmation)
            
        if confirmation not in['yes','jack','juice','yeah']:
            return 'failed'

        add_guest(name_face, drink)
        analyze_face_background(img_face, name_face)
        voice.talk("Nice")

        return 'succ'

# --------------------------------------------------

class Lead_to_living_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        _,guest_name = get_waiting_guests()
        voice.talk(f'{guest_name}... I will lead you to the living room, please follow me')
        # voice.talk('Navigating to ,living room')
        res = omni_base.move_base(known_location='living_room')
        if res:
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'

# --------------------------------------------------

class Find_sitting_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.last_choice = ""
    def execute(self, userdata):

        rospy.loginfo('STATE : Looking for a place to sit')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        voice.talk('I am looking for a place to sit')
        try:
            place, loc = find_empty_places(self.last_choice)
            self.last_choice = place
            print(place, loc)
        except:
            voice.talk('Sorry, there is no more places to sit')
            return 'succ'

        tf_name = place.replace('_', '_face')
        print(tf_name)
        rospy.sleep(1.0)
        head.to_tf(tf_name)

        res=detect_human_to_tf()
        voice.talk('I will check if this place is empty')
        res , _ = wait_for_face()  # seconds
        if res == None:

            print("Place is: ",place)
            _,guest = get_waiting_guests()
            head.set_named_target('neutral')
            rospy.sleep(0.8)
            #head.turn_base_gaze(tf=place)
            head.turn_base_gaze(tf=place, to_gaze='arm_flex_link')
            brazo.set_named_target('neutral')
            voice.talk(f'{guest}, Here is a place to sit')


            assign_occupancy(who=guest, where=place)
            return 'succ'

        else:
            occupant_name = res.Ids.ids
            #print(occupant_name)
            #occupant_name = "someone"
            if occupant_name == 'unknown':
                occupant_name = 'someone'
                #host_name, loc = find_host()
                #occupant_name = host_name
            update_occupancy(found=occupant_name, place=place)
            voice.talk(f'I am sorry, here is {occupant_name}, I will find another place for you')
            return 'failed'

# --------------------------------------------------

class Find_host(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global name_like_host
        rospy.loginfo('STATE : Find host')
        ####Using ANALYZE FACE SERVICE (DEEP FACE SERVER  on FACE_RECOG  PKG)
        # analyze = rospy.ServiceProxy('analyze_face', RecognizeFace)    
        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')

        host_name, loc = find_host()

        if loc == "None":
            #host location is not known
            num_places = len(return_places())
            print(f'num_places: {num_places}')
            name_like_host = ""
            for i in range(1, num_places + 1):
                guest_loc = get_guest_location(name_face)
                guest_face = guest_loc.replace('_','_face')
                tf_name = f'Place_face{i}'
                if guest_face != tf_name:
                    head.to_tf(tf_name)
                    rospy.sleep(1.0)
                    voice.talk(f'looking for host on sit {i}')

                    res, _ = wait_for_face()
                    if res is not None :
                        name = res.Ids.ids
                        if (self.tries == 1) and (name == host_name):
                            self.tries = 0
                            name_like_host = host_name
                            assign_occupancy(who=host_name, where=f'Place_face{i}')
                            return 'succ'
                        elif (self.tries >= 2) and name != 'NO_FACE':
                            if name != 'unknown':
                                name_like_host = name
                            self.tries = 0
                            return 'succ'
                        else:
                            continue
                        #if name != 'NO_FACE':

                        '''takeshi_line = get_guest_description(name_face)                    

                        if takeshi_line != 'None':
                            if name != 'unknown': 
                                speech = f'{name}, {takeshi_line}'
                            else: 
                                speech = takeshi_line
                            timeout = 9.087
                        else:
                            print('no face in img deep analyze')
                            speech = f'{name_face} has arrived'
                            timeout = 5.0
                        voice.talk(speech, timeout)'''
                        '''if len(speech.split(' '))>0: 
                        
                            if speech.split(' ')[4] == 'he':possesive='his'
                            if speech.split(' ')[4] == 'she':possesive='her'
                        rospy.sleep(timeout)
                        voice.talk (f'{possesive} favorite drink is {drink}')
                        rospy.sleep(1.5) '''
                        #self.tries = 0
                        #return 'succ'

            # if the host is not found
            return 'failed'

        else:
            #Host location is known
            name_like_host = host_name
            tf_name = loc.replace('_','_face')
            voice.talk(f'Host is on seat {tf_name.replace("Place_face","")}')
            #rospy.sleep(0.8)
            head.to_tf(tf_name)
            self.tries == 0
            return 'succ'

# --------------------------------------------------
class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global name_like_host
        rospy.loginfo('STATE : Find host')
        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')
        voice.talk(f'Host like name is {name_like_host}')

        takeshi_line = get_guest_description(name_face)                    
        drink = get_guest_drink(name_face)

        if takeshi_line != 'None':
            print("Description found")
            speech = f'{name_like_host}, {takeshi_line}, And likes {drink}'
            timeout = 14.0

        else:
            print('No description found')
            speech = f'{name_like_host}, {name_face} has arrived, And likes {drink}'
            timeout = 7.0


        voice.talk(speech, timeout)
        return 'succ'
        




# --------------------------------------------------
def init(node_name):
    print('smach ready')

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])

    # sm.userdata.clear = False
    #sis = smach_ros.IntrospectionServer(
    #    'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    #sis.start()

    with sm:
        # State machine for Receptionist task

        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           'succ': 'WAIT_PUSH_HAND',   'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    'succ': 'GOTO_DOOR',        'tries': 'WAIT_PUSH_HAND'})
        smach.StateMachine.add("WAIT_DOOR_OPENED",  Wait_door_opened(),     transitions={'failed': 'WAIT_DOOR_OPENED',  'succ': 'GOTO_DOOR',        'tries': 'WAIT_DOOR_OPENED'})
        
        smach.StateMachine.add("SCAN_FACE",         Scan_face(),    transitions={'failed': 'SCAN_FACE',     'succ': 'GET_DRINK',            'tries': 'GOTO_DOOR', 'unknown': 'NEW_FACE',})
        smach.StateMachine.add("NEW_FACE",          New_face(),     transitions={'failed': 'NEW_FACE',      'succ': 'GET_DRINK',            'tries': 'NEW_FACE'})
        smach.StateMachine.add("GET_DRINK",         Get_drink(),    transitions={'failed': 'GET_DRINK',     'succ': 'LEAD_TO_LIVING_ROOM',  'tries': 'GET_DRINK'})

        smach.StateMachine.add("GOTO_DOOR",             Goto_door(),            transitions={'failed': 'GOTO_DOOR',             'succ': 'SCAN_FACE',            'tries': 'SCAN_FACE'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM",   Lead_to_living_room(),  transitions={'failed': 'LEAD_TO_LIVING_ROOM',   'succ': 'FIND_SITTING_PLACE',   'tries': 'END'})
        smach.StateMachine.add("FIND_SITTING_PLACE",    Find_sitting_place(),   transitions={'failed': 'FIND_SITTING_PLACE',    'succ': 'FIND_HOST',            'tries': 'END'})
        smach.StateMachine.add("FIND_HOST",             Find_host(),            transitions={'failed': 'FIND_HOST',             'succ':'INTRODUCE_GUEST',       'tries':'FIND_HOST'})
        smach.StateMachine.add("INTRODUCE_GUEST",       Introduce_guest(),      transitions={'failed': 'INTRODUCE_GUEST',       'succ':'WAIT_PUSH_HAND',        'tries':'WAIT_PUSH_HAND'})


    outcome = sm.execute()
