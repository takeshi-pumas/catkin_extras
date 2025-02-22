#!/usr/bin/env python3

from smach_utils2 import *
from smach_ros import ActionServerWrapper
from action_pickup_floor.msg import FollowAction
##### Define state INITIAL #####

from std_msgs.msg import Bool 
from geometry_msgs.msg import Twist , PointStamped

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
            
        except Exception:
            
            print ('legs _lost')
            talk( 'I lost you, please stand in front of me')
            return 'lost'

        x,y=punto.point.x,    punto.point.y
        
        self.last_legs.append((x,y))
        print(np.linalg.norm(np.asarray(self.last_legs).mean(axis=0)))
        if len (self.last_legs)>=6:
            #if (np.var(self.last_legs,axis=0).mean() < 0.001):
            if (np.linalg.norm(np.asarray(self.last_legs).mean(axis=0)) < 1.0):
                print ('legs stopped... Are we there yet?')#,   np.var(self.last_legs,axis=0).mean()   )     

                talk ('are we there yet ?')
                msg_bool=Bool()
                msg_bool.data= False
                enable_legs.publish(msg_bool)
                enable_follow.publish(msg_bool)
                rospy.sleep(2)
                return 'arrived'
            self.last_legs.pop(0)
        

        
            
        print ('legs moving... Cruising',   np.var(self.last_legs,axis=0).mean()   )          #if (np.var(last_legs,axis=0).mean() < 0.0001):

        return 'succ'

#########################################################################################################
class Goto_next_room(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.next_room=1

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, room'+str(self.next_room))
        res = omni_base.move_base(known_location='room'+str(self.next_room))
        print(res)

        if res == 3:
            (self.tries + 1)%2   #there are 2 rooms <__________________param
            return 'succ'

        else:
            talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Goto_human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        
        print('getting close to human')
        head.to_tf('human')
        res = omni_base.move_d_to(1.5,'human')
        head.to_tf('human')
        print ( "is he drinking?")

        


        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

###########################################################################################################


#########################################################################################################
class Analyze_human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Analyze_human')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        
        
        head.to_tf('human')
        ###image open pose tocayo magic.... #DRINKS
        

        human_pos,_=tf_man.getTF('human')
        head.absolute(human_pos[0],human_pos[1],0.1)

        res=segmentation_server.call()
        ##### SHOES NO SHOES DETECTOR

        human_pos,_=tf_man.getTF('human')
        head.absolute(human_pos[0]+0.1  ,human_pos[1]+0.1,0.1)
        ################### Rubbish on floor nearby?

    
        return 'succ'
    
###########################################################################################################

# --------------------------------------------------
def init(node_name):
    print('smach ready')

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    global enable_legs,enable_follow
    print("Takeshi STATE MACHINE...")

    init("follow_smach")

    enable_legs=  rospy.Publisher('/hri/leg_finder/enable', Bool, queue_size=1)
    enable_follow=rospy.Publisher('/hri/human_following/start_follow', Bool, queue_size=1) 

    sm = smach.StateMachine(outcomes=['SUCCESS','PREEMPTED','FAILED'])

    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/FOLLOW')
    sis.start()


    with sm:
        # State machine for Restaurant
        #smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',       'succ': 'WAIT_PUSH_HAND',   'tries': 'FAILED'})
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',       'succ': 'FIND_LEGS',   'tries': 'FAILED'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND','succ': 'FIND_LEGS',        'tries': 'FAILED'})
        smach.StateMachine.add("FIND_LEGS",          Find_legs(),           transitions={'failed': 'FIND_LEGS',    'succ': 'FOLLOW_HUMAN'    , 'tries': 'FAILED'})
        smach.StateMachine.add("FOLLOW_HUMAN",         Follow_human(),          transitions={'arrived': 'SUCCESS',    'succ': 'FOLLOW_HUMAN'   , 'lost': 'FIND_LEGS'})

        #################################################################################################################################################################
        smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',    'succ': 'GOTO_HUMAN'    , 'tries': 'FIND_LEGS'})
        smach.StateMachine.add("GOTO_HUMAN",         Goto_human(),          transitions={'failed': 'FIND_LEGS',    'succ': 'FIND_LEGS' , 'tries': 'FIND_HUMAN'})
        smach.StateMachine.add("GOTO_NEXT_ROOM",     Goto_next_room(),      transitions={'failed': 'GOTO_NEXT_ROOM','succ': 'FIND_LEGS'    , 'tries': 'FIND_HUMAN'})
        smach.StateMachine.add("ANALYZE_HUMAN",      Analyze_human(),       transitions={'failed': 'FIND_HUMAN',    'succ': 'GOTO_NEXT_ROOM'})
        #################################################################################################################################################################
        ##################################################################################################################################################################
        

        


    asw = ActionServerWrapper(
        'grasp_floor_act_server', FollowAction,
        wrapped_container = sm,
        succeeded_outcomes = ['SUCCESS'],
        aborted_outcomes = ['FAILED'],
        preempted_outcomes = ['PREEMPTED'] )
    asw.run_server()

    outcome = sm.execute()

