#!/usr/bin/env python3

from smach_utils2 import *

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
        if self.tries == 3:
            return 'tries'
            
        clean_knowledge()
        #head.set_named_target('neutral')
        #print('head listo')
        #brazo.set_named_target('go')
        #print('arm listo')
        places_2_tf()
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
        talk('Gently... push my hand to begin')
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
        talk('I am ready for receptionist task.')
        rospy.sleep(0.8)
        talk('I am waiting for the door to be opened')
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

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, door')
        res = omni_base.move_base(known_location='door')
        print(res)

        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

# --------------------------------------------------


class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'unknown', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global  img_face, name_face

        rospy.loginfo('State : SCAN_FACE')
        head.set_joint_values([0.0, 0.3])
        talk('I will scan your face, look at me, please')
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'

        res, img_face = wait_for_face()  # default 10 secs
        rospy.sleep(0.7)
        print (res)

        print('Checking for faces')
        if res != None:
            #return 0

            name = res.Ids.ids

            print('RESPONSE', name)
            if name == 'NO_FACE':
                print('No face Found, Keep scanning')
                talk('I can not see you , I will try again')
                return 'failed'
            elif name == 'unknown':
                print('A face was found.')
                talk('I believe I do not know you')
                return 'unknown'

            else:
                talk(f'I found you, I Think you are, {name}.')
                talk('what do you want to drink?')
                res = speech_recog_server()
                drink = res.data
                name_face=name
                add_guest(name, drink)
                talk('nice')
                takeshi_line = analyze_face_from_image(img_face, name)
                add_description(name, takeshi_line)
                return 'succ'
        else:
            return 'failed'

        # Is this really needed???

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
        #num_waiting_guests, guest_name = get_waiting_guests()
        #if num_waiting_guests == 0:
        if self.tries == 1:
            talk('Please, tell me your name')
            rospy.sleep(0.1)
        else:
            talk('Im sorry, could you tell me your name again?')
        res = speech_recog_server()
        # self.new_name= res.data
        
        try:
            res2 = rospy.wait_for_message( '/recognizedSpeech',RecognizedSpeech, timeout = 5.0)
            print ("POCKERT",res2.hypothesis[0],type(res2.hypothesis))
            name = res.data = res2.hypothesis[0]
        except Exception:   #answer = res2
            return 'failed'
        print (name)
        talk(f'Is {name} your name?')
        res2 = speech_recog_server()
        print (res2)
        rospy.sleep(15.0)
        try:
            res2 = rospy.wait_for_message( '/recognizedSpeech',RecognizedSpeech, timeout = 5.0)
            print ("POCKERT",res2.hypothesis[0],type(res2.hypothesis))
            answer = res2.hypothesis[0]
        except Exception:   #answer = res2
            answer='no'

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
        talk(f'{guest_name}... I will lead you to the living room, please follow me')
        # talk('Navigating to ,living room')
        res = omni_base.move_base(known_location='living_room')
        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

class Goto_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 6:
            return 'tries'
        


        points = rgbd.get_points()   
        res, img_face = wait_for_face()  # default 10 secs
        if res ==None:
            print('no face found...')
            return 'failed'
                                                                ##FACE POS FROM FACE RECOG
        



        boundRect=np.asarray(res.Angs.data).astype('int')       ##FACE POS FROM FACE RECOG RESPONSE AND POINT CLOUD
        trans = bbox_3d_mean(points, boundRect)                 ##FACE POS FROM FACE RECOG
        

        #trans_dict=human_detect_server.call()                   ##FROM HUMAN FINDER OPEN POSE
        #trans =[trans_dict.x,trans_dict.y,trans_dict.z]         ##FROM HUMAN FINDER OPEN POSE
        #print( trans )                                          ##FROM HUMAN FINDER OPEN POSE
        
        #############################################################################################
        ##############################################################################################
        tf_man.pub_static_tf(pos=trans, point_name=name, ref='head_rgbd_sensor_link')
        
        rospy.sleep(0.3)
        tf_man.change_ref_frame_tf(name)
        
        
        try:
            trans,quat = tf_man.getTF(target_frame=name)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No face FOUND')
            return 'failed'
        omni_base.move_d_to(1, name,     )
        head.to_tf(name)


        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

class Find_sitting_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Looking for a place to sit')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        talk('I am looking for a place to sit')
        place, loc = find_empty_places()
        print(place, loc)
        #omni_base.move_base(*loc, time_out = 15)
        #head.set_named_target("neutral")
        
        #gaze each place, instead of navigating
        #[location,num]=place.split("_")
        #tf_name = f'{location}_face{num}'
        tf_name = place.replace('_', '_face')
        print(tf_name)
        rospy.sleep(1.0)
        head.to_tf(tf_name)

        res=detect_human_to_tf()
        #print (res,'Human')
        #talk('I will check if there is no people on this sit')
        res , _ = wait_for_face()  # seconds
        #print (res,'face')
        if res == None:

            _,guest = get_waiting_guests()
            head.set_named_target('neutral')
            head.turn_base_gaze(tf=place)
            brazo.set_named_target('neutral')
            talk(f'{guest}, Here is a place to sit')
            print(place)
            assign_occupancy(who=guest, where=place)
            return 'succ'

        else:
            occupant_name = res.Ids.ids
            #print(occupant_name)
            #occupant_name = "someone"
            if occupant_name == 'unknown':
                host_name, loc = find_host()
                occupant_name = host_name
            update_occupancy(found=occupant_name, place=place)
            talk(f'I am sorry, here is {occupant_name}, I will find another place for you')
            return 'failed'

# --------------------------------------------------
class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Introduce_guest')
        ####Using ANALYZE FACE SERVICE (DEEP FACE SERVER  on FACE_RECOG  PKG)
        # analyze = rospy.ServiceProxy('analyze_face', RecognizeFace)    
        self.tries += 1
        print('Try', self.tries, 'of 3 attempts')
        # if self.tries == 2:
            # omni_base.move_base(known_location='living_room')
            # rospy.sleep(1.0)


        host_name, loc = find_host()
        #host location is not known
        '''name = ''
        if loc == "None":
            #talk("i dont know where the host is")
            num_places = len(return_places())
            print(f'num_places: {num_places}')
            for i in range (1, num_places + 1):
                head.to_tf(f'Place_face{i}')
                talk(f'looking for host in place {i}')
                res, _ = wait_for_face()

                if res != None:
                    name = res.Ids.ids
                    #If someone is found
                    if name != 'NO_FACE':
                        takeshi_line = get_guest_description(name_face)
                        if name != 'unknown':
                            takeshi_line = f'{name}, {takeshi_line}'

                        print (takeshi_line)
                        if takeshi_line!=None:
                            talk(takeshi_line)
                            rospy.sleep(7.0)
                        else :
                            print ('no face in img deep analyze')
                            talk(f'{name_face} has arrived')
                        self.tries = 0
                        return 'succ'
                    elif name == 'NO_FACE' and i == num_places:
                        return 'failed'
                #host_name = res.Ids.ids[0].data
                rospy.sleep(1.0)
        else:
            [place,num]=loc.split("_")
            tf_name = f'{place}_face{num}'
            print(tf_name)
            rospy.sleep(1.0)
            head.to_tf(tf_name)'''
        #host location is not known

        if loc == "None":
            #talk("i dont know where the host is")
            num_places = len(return_places())
            print(f'num_places: {num_places}')
            for i in range(1, num_places + 1):
                guest_loc = get_guest_location(name_face)
                guest_face = guest_loc.replace('_','_face')
                tf_name = f'Place_face{i}'
                if guest_face != tf_name:
                    head.to_tf(tf_name)
                    talk(f'looking for host on sit {i}')
                    res, _ = wait_for_face()

                    if res is not None and res.Ids.ids != 'NO_FACE':
                        name = res.Ids.ids
                        takeshi_line = get_guest_description(name_face)                    

                        if takeshi_line != 'None':
                            if name != 'unknown': 
                                speech = f'{name}, {takeshi_line}'
                            else: 
                                speech = takeshi_line
                            timeout = 7.0
                        else:
                            print('no face in img deep analyze')
                            speech = f'{name_face} has arrived'
                            timeout = 1.0
                        talk(speech)
                        rospy.sleep(timeout)
                        self.tries = 0
                        return 'succ'

            # if the host is not found
            return 'failed'

        else:
            tf_name = loc.replace('_','_face')
            head.to_tf(tf_name)
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
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()

    with sm:
        # State machine for Restaurant

        smach.StateMachine.add("INITIAL",           Initial(),          transitions={'failed': 'INITIAL',       'succ': 'WAIT_PUSH_HAND',   'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions={'failed': 'WAIT_PUSH_HAND','succ': 'GOTO_DOOR',        'tries': 'INITIAL'})
        smach.StateMachine.add("WAIT_DOOR_OPENED",  Wait_door_opened(),   transitions={'failed': 'WAIT_DOOR_OPENED','succ': 'GOTO_DOOR',        'tries': 'INITIAL'})
        
        smach.StateMachine.add("SCAN_FACE",         Scan_face(),        transitions={'failed': 'SCAN_FACE',     'unknown': 'NEW_FACE',      'succ': 'LEAD_TO_LIVING_ROOM','tries': 'GOTO_DOOR'})
        smach.StateMachine.add("NEW_FACE",          New_face(),         transitions={'failed': 'NEW_FACE',      'succ': 'LEAD_TO_LIVING_ROOM','tries': 'NEW_FACE'})
        smach.StateMachine.add("GOTO_DOOR",         Goto_door(),        transitions={'failed': 'GOTO_DOOR',     'succ': 'SCAN_FACE',        'tries': 'SCAN_FACE'})
        smach.StateMachine.add("GOTO_FACE",         Goto_face(),        transitions={'failed': 'GOTO_FACE',     'succ': 'LEAD_TO_LIVING_ROOM',   'tries': 'SCAN_FACE'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM",Lead_to_living_room(), transitions={'failed': 'LEAD_TO_LIVING_ROOM','succ': 'FIND_SITTING_PLACE','tries': 'END'})
        smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(),  transitions={'failed': 'FIND_SITTING_PLACE','succ': 'INTRODUCE_GUEST',    'tries': 'END'})
        smach.StateMachine.add("INTRODUCE_GUEST",   Introduce_guest(),      transitions={'failed':'INTRODUCE_GUEST','succ':'WAIT_PUSH_HAND',    'tries':'WAIT_PUSH_HAND'})

    outcome = sm.execute()
