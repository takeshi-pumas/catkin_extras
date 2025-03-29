#!/usr/bin/env python3
# Author: Oscar
# ofc1227@tec.mx
#from smach_utils2 import *
from utils_follow import *

from smach_ros import ActionServerWrapper
from action_server.msg import FollowAction
##### Define state INITIAL #####

from std_msgs.msg import Bool 
from geometry_msgs.msg import Twist , PointStamped



# --------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        
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
        print('Detecting Humans ')
        if self.tries==1:head.set_joint_values([ 0.0, 0.3])
        if self.tries==2:head.set_joint_values([ 0.7, 0.1])
        if self.tries==3:head.set_joint_values([-0.7, 0.1])

        
        rospy.sleep(0.7)
        humanpose=detect_human_to_tf()



        if humanpose== False:
            print ('no human ')
            return 'failed'
        else : 

            head.to_tf('human')
            print ('human tf published')
            return 'succ'    

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
class Goto_human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.last_legs=[]
    def execute(self, userdata):
        rospy.loginfo('STATE : Human_found,Navigating to last know position')
        pose_robot,quat_robot= tf_man.getTF('base_link')
        head.set_joint_values([0.0,-0.7 ])
        rospy.sleep(0.75)
        yaw_robot=tf.transformations.euler_from_quaternion(quat_robot)[2]
        print (pose_robot[:2], yaw_robot, 'robot pose')
        tf_man.pub_static_tf(np.asarray((1.0,0.0,0.0)),point_name='move_base_goal', ref='human')
        res=move_base(1.00,0.0,yaw_robot,'human')
        head.to_tf('human')
        print (res)
        if res:
            return 'succ'




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


    enable_legs = rospy.Publisher('/hri/leg_finder/enable', Bool, queue_size=1)
    enable_follow = rospy.Publisher('/hri/human_following/start_follow', Bool, queue_size=1)

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])

    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/FOLLOW')
    sis.start()

    with sm:
        # Initialization state
    
        
        # Other states
        smach.StateMachine.add("INITIAL", Initial(), transitions={'failed': 'INITIAL', 'succ': 'FIND_LEGS'})
        smach.StateMachine.add("WAIT_PUSH_HAND", Wait_push_hand(), transitions={'failed': 'WAIT_PUSH_HAND', 'succ': 'FIND_LEGS', 'tries': 'failed'})
        smach.StateMachine.add("FIND_HUMAN", Find_human(), transitions={'failed': 'FIND_HUMAN', 'succ': 'GOTO_HUMAN', 'tries': 'failed'})
        smach.StateMachine.add("FIND_LEGS", Find_legs(), transitions={'failed': 'FIND_LEGS', 'succ': 'FOLLOW_HUMAN', 'tries': 'failed'})
        smach.StateMachine.add("GOTO_HUMAN", Goto_human(), transitions={'failed': 'FIND_HUMAN', 'succ': 'succeeded', 'tries': 'failed'})
        smach.StateMachine.add("FOLLOW_HUMAN", Follow_human(), transitions={'arrived': 'succeeded', 'succ': 'FOLLOW_HUMAN', 'lost': 'FIND_LEGS'})

    asw = ActionServerWrapper(
        'follow_server', FollowAction,
        wrapped_container=sm,
        succeeded_outcomes=['succeeded'],
        aborted_outcomes=['failed'],
        preempted_outcomes=['preempted'])
    asw.run_server()



# --------------------------------------------------
def init(node_name):
    print('smach ready')

# --------------------------------------------------
# Entry point
