#!/usr/bin/env python3
# Author: Oscar
# ofc1227@tec.mx
import rospy
import actionlib
from action_server.msg import IdentifyPersonAction, IdentifyPersonResult, IdentifyPersonFeedback
from smach_ros import ActionServerWrapper
from utils_action import *
head = GAZE()



# --------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        rospy.loginfo(f"Received command: {sm.userdata.goal.command}")
        #print (f'self.sm.userdata.goal.mode -> {self.sm.userdata.goal.mode.data}')
        head.set_named_target('neutral')
        #print('head listo')
        #brazo.set_named_target('go')
        #print('brazo listo')
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
        talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'


# --------------------------------------------------


class Find_human(smach.State):
    def __init__(self):
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
        if self.tries==1:head.set_joint_values([ 0.0, 0.3])
        if self.tries==2:head.set_joint_values([ 0.7, 0.1])
        if self.tries==3:head.set_joint_values([-0.7, 0.1])

        
        rospy.sleep(0.7)
        humanpose=detect_human_to_tf()

        print('Detecting Humans ')


        if humanpose== False:
            print ('no human ')
            return 'failed'
        else : 
            head.to_tf('human')
            return 'succ'    

class Find_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ['succ', 'failed','tries'], 
                             output_keys = ['name', 'face_img'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('State : Find face')
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        if self.tries==1:head.set_joint_values([ 0.0, 0.3])
        if self.tries==2:head.set_joint_values([ 0.7, 0.1])
        if self.tries==3:head.set_joint_values([-0.7, 0.1])
        talk('Waiting for guest, look at me, please')
        res, userdata.face_img = wait_for_face()  # default 10 secs
        #rospy.sleep(0.7)
        if res != None:
            #userdata.name = res.Ids.ids
            return 'succ'
        else:
            return 'failed'

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
            talk('I did not see you, I will try again')
            return 'failed'

        elif userdata.name == 'unknown':
            talk('I believe we have not met.')
            self.tries = 0
            return 'unknown'

        else:
            talk(f'I found you, I Think you are, {userdata.name}.')
            talk('Is it correct?')
            #rospy.sleep(2.5)
            confirmation = get_keywords_speech(10)
            print (confirmation)

            if confirmation not in ['yes','jack','juice', 'takeshi yes','yeah']:
                talk(f'I am looking for{sm.userdata.goal.command} ')
                return 'unknown'
            elif confirmation == "timeout":
                talk('I could not hear you, lets try again, please speak louder.')
                return "failed"
            else:       
                talk(f'{sm.userdata.goal.command} Found.')
                         
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
            talk ('I didnt undestand your name, lets continue')
            userdata.name = 'someone'
            train_face(userdata.face_img, userdata.name )
            party.add_guest(userdata.name)
            self.tries = 0
            return 'succ'
        
        #Asking for name
        talk('Please, tell me your name')
        #rospy.sleep(1.0)
        speech = get_keywords_speech(10)
        # in case thinks like I am , my name is . etc
        if len(speech.split(' ')) > 1: name = (speech.split(' ')[-1])
        else: name = speech

        if userdata.name == 'timeout':
            talk('Please repeat it and speak louder.')
            return 'failed'

        talk(f'Is {name} your name?')
        #rospy.sleep(2.0)
        confirmation = get_keywords_speech(10)
        print (confirmation)

        confirmation = confirmation.split(' ')
        confirm = match_speech(confirmation, ['yes','yeah','jack','juice'])
        if confirm:
            userdata.name = name
            talk (f'Nice to Meet You {userdata.name}')
            party.add_guest(userdata.name)
            train_face(userdata.face_img, userdata.name)
            self.tries = 0
            return 'succ'
        else:
            talk ('lets try again')
            return 'failed'






class Find_legs(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0   

    def execute(self, userdata):
        self.tries+=1
        if self.tries==6:
            self.tries=0
            return 'tries'
        
        #ENABLE LEG FINDER AND HUMAN FOLLOWER
        msg_bool=Bool()
        msg_bool.data= True
        enable_legs.publish(msg_bool)
        enable_follow.publish(msg_bool)
        ############################
        talk('Leg finder activated')
        print ('Leg finder activated ')
        
        timeout=2
        if self.tries==1:timeout=0.1####FIRST ENABLE TAKES A WHILE
        
        try :
            punto=rospy.wait_for_message("/hri/leg_finder/leg_pose", PointStamped , timeout=timeout)
            print (punto)
            self.tries=0
            return 'succ'
            
            

        except Exception:
            
            print ('No legs found')
            talk( 'I can not  find you, please stand in front of me')
      
            
            msg_bool.data= False
            enable_legs.publish(msg_bool)
            enable_follow.publish(msg_bool)

            return 'failed'    

#########################################################################################################
class Follow_human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'arrived', 'lost'])
        self.tries = 0
        self.last_legs=[]

    def execute(self, userdata):

        rospy.loginfo('STATE : Legs_found,following')
        self.tries+=1
        if self.tries == 1: 
            print('Legs Found, Following')
            talk('Human found, Following')

        try :
            punto=rospy.wait_for_message("/hri/leg_finder/leg_pose", PointStamped , timeout=2.0)
            pub_goal.publish(punto)
        except Exception:            
            print ('legs _lost')
            talk( 'I lost you, please stand in front of me')
            return 'lost'
        x,y=punto.point.x,    punto.point.y        
        self.last_legs.append((x,y))
        print(np.linalg.norm(np.asarray(self.last_legs).mean(axis=0)))
        if len (self.last_legs)>=26:
            #if (np.var(self.last_legs,axis=0).mean() < 0.001):
            if (np.linalg.norm(np.asarray(self.last_legs).mean(axis=0)) < 1.0):
                print ('legs stopped... Are we there yet?')#,   np.var(self.last_legs,axis=0).mean()   )    
                talk ('are we there yet ?')
                print ('are we there yet ?')                
                speech = get_keywords_speech(10)
                speech = speech.split(' ')
                confirmation_list=['yes','jack','juice', 'takeshi yes','yeah']
                confirm = any(word in confirmation_list for word in speech)
                
                print (speech,"#################################################",confirm)               
                if confirm:
                    talk ('arrival confirmed, exiting action')
                    print ('We are athere')
                    msg_bool=Bool()
                    msg_bool.data= False
                    enable_legs.publish(msg_bool)
                    enable_follow.publish(msg_bool)
                    return 'arrived' 
                    
                talk ('ok then ,  I will continue to follow you')
                print('ok I will continue to follow you')
            self.last_legs.pop(0)
        print ('legs moving... Cruising',   np.var(self.last_legs,axis=0).mean()   )          #if (np.var(last_legs,axis=0).mean() < 0.0001):
        return 'succ'



if __name__ == '__main__':
    global enable_legs, enable_follow
    print("Takeshi STATE MACHINE...")


    

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'],input_keys=["goal"])
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/ID_PERSON')
    sis.start()

    with sm:
        # Initialization state
    
        
        # Other states
        smach.StateMachine.add("INITIAL", Initial(), transitions={'failed': 'INITIAL', 'succ': 'FIND_HUMAN'})
        smach.StateMachine.add("FIND_HUMAN", Find_human(), transitions={'failed': 'FIND_HUMAN', 'succ': 'FIND_FACE', 'tries': 'failed'})
        smach.StateMachine.add("FIND_FACE" , Find_face() , transitions={'failed': 'FIND_FACE' , 'succ': 'DECIDE_FACE', 'tries': 'failed'})
        smach.StateMachine.add("DECIDE_FACE", Decide_face(),transitions={'failed': 'FIND_FACE', 'succ': 'succeeded', 'unknown': 'NEW_FACE'})
        smach.StateMachine.add("NEW_FACE", New_face(),transitions={'failed': 'NEW_FACE', 'succ': 'succeeded'})
        
        smach.StateMachine.add("WAIT_PUSH_HAND", Wait_push_hand(), transitions={'failed': 'WAIT_PUSH_HAND', 'succ': 'FIND_HUMAN', 'tries': 'failed'})
        smach.StateMachine.add("FIND_LEGS", Find_legs(), transitions={'failed': 'FIND_LEGS', 'succ': 'FOLLOW_HUMAN', 'tries': 'failed'})
        smach.StateMachine.add("FOLLOW_HUMAN", Follow_human(), transitions={'arrived': 'succeeded', 'succ': 'FOLLOW_HUMAN', 'lost': 'FIND_LEGS'})

    asw = ActionServerWrapper(
        'identify_person_server', IdentifyPersonAction,
        wrapped_container=sm,
        succeeded_outcomes=['succeeded'],
        aborted_outcomes=['failed'],
        preempted_outcomes=['preempted'],
        goal_key='goal')
    
    asw.run_server()




        