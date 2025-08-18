#!/usr/bin/env python3
from restaurant_utils import *
#from smach_utils_give_me_a_hand import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

# Initial STATE: task setup (grammar, knowledge, robot position, ...)

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo('STATE : INITIAL')
        rospy.loginfo(f'Try {self.tries} of 5 attempts')
        global seat_places
        #party.clean_knowledge(host_name = "Oscar", host_location = "Place_3")
        #party.publish_tf_seats()
        #places_2_tf()
        df=yaml_to_df()
        seat_places = df[df['child_id_frame'].str.startswith('seat_place_')]
        for index, row in seat_places.iterrows():
            x = row['x']
            y = row['y']
            z = 1.0
            child_id = row['child_id_frame']
            print(f"{child_id}: x = {x}, y = {y}, z = {z}")
            tf_man.pub_static_tf(pos=[x,y,1.0],rot=[0,0,0,1],point_name=child_id)

        print(seat_places)
        ###-----INIT GRAMMAR FOR VOSK
        ###-----Use with get_keywords_speech()
        ###-----------SPEECH REC
        #drinks=['coke','juice','beer', 'water', 'soda', 'wine', 'i want a', 'i would like a']
        #drinks = ['coke','juice','milk', 'water', 'soda', 'wine', 
        #          'i want a', 'i would like a', 'tea', 'icedtea', 'cola', 'redwine', 'orangejuice', 'tropicaljuice']
       
        if self.tries == 1:
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
        talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'

# Wait door opened STATE: Trigger for task to start

class Wait_door_opened(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.first = True
        self.tries = 0

    def execute(self, userdata):
        if self.first:
            self.first=False 
            rospy.loginfo('STATE : Wait for door to be opened')
            print('Waiting for door to be opened')
            self.tries += 1
            print(f'Try {self.tries} of 4 attempts')

            # if self.tries == 100:
            #     return 'tries'
            talk('I am ready for give me a hand task.')
            rospy.sleep(0.8)
            talk('I am waiting for the door to be opened')
            succ = line_detector.line_found()
            #succ = wait_for_push_hand(100)
            rospy.sleep(1.0)
            if succ:
                self.tries = 0
                return 'succ'
            else:
                return 'failed'
        else:
            return 'succ'

class Go_to_instruction_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : navigate to instruction area')
        print('Try', self.tries, 'of 3 attempts')
        if  self.tries >=1:
            res = omni_base.move_base(known_location_2='instruction_point')
            res = omni_base.move_base(known_location='instruction_point')
            if res:
                self.tries=0
                return 'succ'
            else:
                talk('Navigation Failed, retrying')
                return 'failed'
        if self.tries==0:
            res = omni_base.move_base(known_location='instruction_point')
            if res:
                self.tries += 1
                return 'succ'
            else:
                talk('Navigation Failed, retrying')
                return 'failed'
        
# find operator to interact withs
#class Wait_for_waving(smach.State):
""" def __init__(self):
                 smach.State.__init__(
                     self, outcomes=['succ', 'failed', 'tries'])
                 self.tries = 0
         
             def execute(self, userdata):
                 
         
                 rospy.loginfo('State : Find_human')
                 talk('Scanning the room for humans')
                 self.tries += 1
                 if self.tries >= 4:
                     self.tries = 0
                     return'tries'
         
                 if self.tries==1:head.set_joint_values([ 0.0, 0.0])
                 if self.tries==2:head.set_joint_values([ 0.3, 0.1])
                 if self.tries==3:head.set_joint_values([-0.3, 0.1])
         
                 
                 rospy.sleep(0.7)
                 humanpose=detect_human_to_tf()
         
                 print('Detecting Humans ')
         
         
                 if humanpose== False:
                     print ('no human ')
                     return 'failed'
                 else : 
                     talk('Human Found, Getting close ')
                     head.to_tf('human')
                     self.tries=0
                     return 'succ'    
         """

class Wait_for_waving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0
        self.gaze = [[ 0.0, 0.05],[ 0.0, 0.05],[ -0.5, 0.15],[ 0.5, 0.15],[ 0.85, 0.2],[ -0.85, 0.2]]
    def execute(self, userdata):
        #req = RecognizeRequest()    # DOCKER
        req = RecognizeOPRequest() # NORMAL

        # Guardo posicion para retornar
        trans, quat=tf_man.getTF('base_link')
        tf_man.pub_static_tf(pos=trans, rot =[0,0,0,1], point_name='INITIAL_PLACE')
        rospy.loginfo('STATE : Wait for a person waving') 
        if self.tries < 6:self.tries += 1
        else: self.tries=0
        
        print(f'[WAITFORWAVING]Try {self.tries} of N attempts')
        
        talk('Looking for a person waving')
        print('[WAITFORWAVING] Looking for a waving person')
        print(f"[WAITFORWAVING] gaze:{self.gaze[self.tries]}")
        head.set_joint_values(self.gaze[self.tries])

        rospy.sleep(1)
        req.visual=0
        # reqAct.in_ --> 5  para detectar Waving en Restaurant
        req.in_ = 5
        talk("Three")
        print("[WAITFORWAVING] 3")
        #rospy.sleep(1.0)
        talk('Two')
        print("[WAITFORWAVING] 2")
        #rospy.sleep(1.0)
        talk('One')
        print("[WAITFORWAVING] 1")
        #rospy.sleep(1.0)

        #resAct=recognize_action_docker(req) # DOCKER
        resAct=recognize_action(req)       # NORMAL
        
        print("[WAITFORWAVING] RES OF SERVICE:",resAct.i_out)


        #img = bridge.imgmsg_to_cv2(resAct.im_out.image_msgs[0])
        save_image(bridge.imgmsg_to_cv2(resAct.im_out.image_msgs[0]),name="wavingRestaurant")
        if resAct.i_out == 1:
            talk('I found someone waving')
            print("[WAITFORWAVING] I found someone waving")
        else:
            talk('I did not found someone waving')
            print("[WAITFORWAVING] I did not found someone waving")
            return 'failed' if self.tries < 6 else 'tries'
        
        

        talk('Aproaching to the waving person')
        print('[WAITFORWAVING] Aproaching to the waving person')
        self.tries=0
        return 'succ'
#--------------------------------------------------
class Goto_waving_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Navigate to forbidden room')

        print(f'[GOTOWAVINGPERSON] Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        
        human_xyz,_=tf_man.getTF('person_waving',ref_frame='base_link')
        print(f'[GOTOWAVINGPERSON] coords {human_xyz}')

        res = omni_base.move_d_to(0.7,'person_waving')
        # publica pointStamp para que se mueva con potFields o lo que se vaya a usar
        #res = new_move_D_to(tf_name='person_waving',d_x=1 , timeout=20.0)
        print("[GOTOWAVINGPERSON]",res)

        if res:
            # move_base() # with no map
            self.tries = 0
            talk('Arrrived to person')
            rospy.sleep(1)
            return 'succ'
        else:
            print('[GOTOWAVINGPERSON] Navigation Failed, retrying')
            return 'failed'

class Grasp_object(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['grasped_item', 'failed'],
            # input_keys=['available_items', 'order'],
            output_keys=['place_spot']
        )

    def execute(self, userdata):
        # Step 1: Find wrist position
        res=wrist_detect_server.call()
        print (f'x{res.x} y{res.y} z{res.z} ')
        object_loc = [res.x, res.y, res.z]

        tf_man.pub_static_tf(pos=object_loc, rot =[0,0,0,1], point_name='object_location')
        rospy.sleep(0.5)

        arm.set_named_target('neutral')      
        arm.go() 
        gripper.open()   
        res = omni_base.move_d_to(0.5,'object_location')
        gripper.close()

        talk("Where would you like me to place the object?")
        print ("Where would you like me to place the object?")

        speech = get_keywords_speech(10)

        if speech == 'timeout':
            talk(' Sorry, I did not hear you, please repeat it and speak louder.')

        userdata.place_spot = speech.lower().split()

        return 'grasped_item' 
    
class Go_to_place_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'],input_keys=['place_spot'], output_keys=['words'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : navigate to place area')
        print('Try', self.tries, 'of 3 attempts')

        place_area = "dishwasher"

        if "dishwasher" in userdata.words():
            place_area = "dishwasher"
        elif "table" in userdata.words():
            place_area == "kitchen_table"
        elif "frige" in userdata.words():
            place_area = "refrigerator" 
        elif "sink" in userdata.words():
            place_area = "sink"

        res = omni_base.move_base(known_location=place_area)
        if res:
            self.tries = 0
            # return 'succ'
            talk("Could you clarify verbally where exactly would you like me to place the item?")
            print ("Could you clarify verbally where exactly would you like me to place the item?")

            speech = get_keywords_speech(10)

            if speech == 'timeout':
                talk(' Sorry, I did not hear you, please repeat it and speak louder.')

            userdata.words = speech.lower().split()

            return 'succ'

        else:
            talk('Navigation Failed, retrying')
            return 'failed'

class GetPlacingTF(smach.State):   
    def __init__(self):
        smach.State.__init__(self, input_keys=['words'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : GET_PLACING_TF')
        try:
            _,res = get_placing_tf(userdata.words)
            if res:
                talk('Placing...')
                return 'succ'
            else:
                talk('I failed, please take the object')
                brazo.set_named_target("neutral")
                gripper.close()
                rospy.sleep(1.0)
                talk("In three...")
                rospy.sleep(0.2)
                talk("Two...")
                rospy.sleep(0.2)
                talk("One...")
                rospy.sleep(0.2)
                gripper.open()
                rospy.sleep(2)
                gripper.steady()
                return 'failed'
        except Exception as e:
            print(e)
            return 'succ'

       

        
class Place(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        self.tries +=1
        global cat,target_object
        rospy.loginfo('STATE : PLACE')
        if self.tries == 3:
            brazo.set_named_target("neutral")
            talk('I failed, please take the object')
            gripper.close()
            rospy.sleep(1.0)
            talk("In three...")
            rospy.sleep(0.2)
            talk("Two...")
            rospy.sleep(0.2)
            talk("One...")
            rospy.sleep(0.2)
            gripper.open()
            rospy.sleep(2)
            gripper.steady()
            return 'tries'
        pos, _ = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'odom')
        target_pose = Float32MultiArray()
        pos[2] += 0.05
        target_pose.data = pos
        userdata.target_pose = target_pose

        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        
        #clear octo client is recommended
        clear_octo_client()               

        return 'succ'  

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    sm.userdata.guest_num=1
    #sis = smach_ros.IntrospectionServer(
    #    'SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    #sis.start()

    with sm:
        # State machine 

        # Initial states routine
        smach.StateMachine.add("INITIAL", Initial(),              
                               transitions={'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND'})
                                #'succ': 'SCAN_FACE'})
        smach.StateMachine.add("WAIT_PUSH_HAND", Wait_push_hand(),       
                               transitions={'failed': 'WAIT_PUSH_HAND'   #, 'succ':'SCAN_FACE'
                               ,'succ':'WAIT_DOOR_OPENED'
                               })
        smach.StateMachine.add("WAIT_DOOR_OPENED", Wait_door_opened(),     
                               transitions={'failed': 'WAIT_DOOR_OPENED','succ': 'GO_TO_INSTRUCTION_ROOM'})

        # Interaction
        smach.StateMachine.add("GO_TO_INSTRUCTION_ROOM", Go_to_instruction_room(),  
                               transitions={'failed': 'GO_TO_INSTRUCTION_ROOM', 'succ': 'WAITING_WAVING'})
        
        smach.StateMachine.add("WAITING_WAVING", Wait_for_waving(),            
                               transitions={'failed': 'WAITING_WAVING', 'succ': 'GOTO_WAVING_PERSON', 'tries':'GO_TO_INSTRUCTION_ROOM'})
        
        
        smach.StateMachine.add("GOTO_WAVING_PERSON", Goto_waving_person(),    
                               transitions={'failed': 'GOTO_WAVING_PERSON', 'succ': 'GET_OBJECT','tries':'GO_TO_INSTRUCTION_ROOM'})
        
        smach.StateMachine.add("GET_OBJECT", Grasp_object(),            
                               transitions={'failed': 'GET_OBJECT', 'grasped_item': 'GO_PLACE'})
        smach.StateMachine.add("GO_PLACE", Go_to_place_position(),            
                               transitions={'failed': 'GO_PLACE', 'succ': 'GET_PLACING_TF'})
        smach.StateMachine.add("PLACE_OBJECT", Go_to_place_position(),            
                               transitions={'failed': 'PLACE_OBJECT', 'succ': 'GO_TO_INSTRUCTION_ROOM'})
        smach.StateMachine.add("GET_PLACING_TF", GetPlacingTF(),            
                               transitions={'failed': 'GO_TO_INSTRUCTION_ROOM', 'succ': 'PLACE', 'tries': 'GO_TO_INSTRUCTION_ROOM'})
        smach.StateMachine.add("PLACE",    Place(),                     transitions={'failed': 'GET_PLACING_TF',    
                                                                                         'succ': 'PLACE_GOAL',
                                                                                         'tries': 'GO_TO_INSTRUCTION_ROOM'})
        smach.StateMachine.add("PLACE_GOAL", SimpleActionState('place_server', GraspAction, goal_slots=['target_pose']),              
                        transitions={'preempted': 'GET_PLACING_TF', 'succeeded': 'GO_TO_INSTRUCTION_ROOM', 'aborted': 'PLACE'})
        
        


    outcome = sm.execute()