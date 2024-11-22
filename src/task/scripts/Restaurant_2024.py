#!/usr/bin/env python3
from restaurant_utils import *


#--------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo('STATE : INITIAL')
        rospy.loginfo(f'[INITIAL] Try {self.tries} of 5 attempts')

        if self.tries == 1:
            #set_grammar(gram)  ##PRESET DRINKS
            rospy.sleep(0.2)
            return 'succ'
        elif self.tries == 3:
            return 'failed'
        
#--------------------------------------------------
# Wait push hand STATE: Trigger for task to start
class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Wait for Wait_push_hand')
        self.tries += 1
        print(f'[WAITPUSHHAND] Try {self.tries} of 4 attempts')
        # if self.tries == 4:
        #     return 'failed'
        #head.set_named_target('neutral')
        #brazo.set_named_target('go')
        talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(50)

        if succ:
            return 'succ'
        else:
            return 'failed'

#--------------------------------------------------
# Wait for someone to start waving
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

#--------------------------------------------------
class Demo_logic(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Logic for the demo ')

        print(f'[DEMOLOGIC] Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        
        head.set_joint_values([0.0, 0.4])
        print(f'[DEMOLOGIC] Talking....')

        talk('Hello, I saw that you were waving')
        rospy.sleep(0.5)
        talk('My name is Takeshi, I will lead you to my lab stand, please follow me')
        rospy.sleep(0.9)
        return 'succ'
        
        
#--------------------------------------------------
class Return_to_Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Returning to initial point')
        req = RecognizeOPRequest() # NORMAL
        req.visual=0
        # reqAct.in_ --> 5  para detectar Waving en Restaurant
        req.in_ = 5

        print(f'[RETURNTOINITIAL] Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 5:
            return 'tries'
        print("[RETURNTOINITIAL] head to tf...")
        
        res = head.to_tf('INITIAL_PLACE')
    
        head.set_joint_values([0.0,0.1])
        rospy.sleep(3.8)
        talk("Checking...")
        resAct=recognize_action(req)       
        print(f"[RETURNTOINITIAL] RES DE CHECKING {resAct.i_out}")
        if resAct.i_out!=1:
            talk("Retrying")
            return 'failed'
        talk("ok")


        res = omni_base.move_d_to(0.5,'person_waving')
        #res = omni_base.move_base(known_location='INITIAL_PLACE')
        rospy.sleep(1)
        omni_base.tiny_move(velT = 1.0, std_time = 3.5, MAX_VEL_THETA= 1.0 )

        

        if res:
            # move_base() # with no map
            rospy.sleep(1)
            talk('Arrived..')
            rospy.sleep(1)
            return 'succ'
        else:
            print('[RETURNTOINITIAL] Navigation Failed, retrying')
            return 'failed'

#--------------------------------------------------
class Final_part_demo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Logic for the demo ')

        print(f'[FINALPARTDEMO] Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        
        head.set_joint_values([0.0, 0.4])
        print(f'[FINALPARTDEMO] Talking....')

        talk('Done. Feel free to ask anything about me or my research lab to them, thank you.')
        rospy.sleep(2.5)
        return 'succ'
      


# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("[MAIN]Takeshi STATE MACHINE...")
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
                               transitions={'failed': 'WAIT_PUSH_HAND', 'succ': 'WAITING_WAVING'})

        smach.StateMachine.add("WAITING_WAVING", Wait_for_waving(),            
                               transitions={'failed': 'WAITING_WAVING', 'succ': 'GOTO_WAVING_PERSON', 'tries':'END'})
        
        
        smach.StateMachine.add("GOTO_WAVING_PERSON", Goto_waving_person(),    
                               transitions={'failed': 'GOTO_WAVING_PERSON', 'succ': 'DEMO_LOGIC','tries':'END'})
        
        smach.StateMachine.add("DEMO_LOGIC", Demo_logic(),    
                               transitions={'failed': 'DEMO_LOGIC', 'succ': 'RETURN_TO_INITIAL','tries':'END'})
        
        smach.StateMachine.add("RETURN_TO_INITIAL", Return_to_Initial(),    
                               transitions={'failed': 'RETURN_TO_INITIAL', 'succ': 'FINAL_PART_DEMO','tries':'END'})
        
        smach.StateMachine.add("FINAL_PART_DEMO", Final_part_demo(),    
                               transitions={'failed': 'END', 'succ': 'WAIT_PUSH_HAND','tries':'END'})
        

    outcome = sm.execute()
