#!/usr/bin/env python3
import sys
import smach
import rospy
import cv2 as cv
import numpy as np
from std_srvs.srv import Empty
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from utils.grasp_utils import *
from utils.nav_utils import *

##### Define state INITIAL #####

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : INITIAL')
        print('robot neutral pose')
        print('Try',self.tries,'of 5 attempts') 
        self.tries+=1
        if self.tries==3:
            return 'tries'
        arm.set_named_target('go')
        head.set_named_target('neutral')       
        arm.go()
        head.go()
        talk('I am ready')
        rospy.sleep(1.0)
        return 'succ'

class Wait_door(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('STATE : Wait_door')
        print('robot neutral pose')
        print('Try',self.tries,'of 100 attempts') 
        self.tries+=1
        rospy.sleep(1.0)
        if self.tries == 1:
            talk('I am waiting for door')
            rospy.sleep(0.7)

        if not line_detector.line_found():
            rospy.sleep(0.7)
            talk('I can see the door is opened, entering')
            rospy.sleep(0.7)
            return 'succ'
        else:
            return 'tries'


class Goto_area(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
        
    def execute(self,userdata):
        rospy.loginfo('STATE : GOTO_AREA')
        self.tries+=1
        # head.set_joint_value_target([0.0,-0.5])
        # head.go()
        # rospy.sleep(1.0)
        if self.tries==5:
            return 'tries'
        base.move_base(known_location='living_room')
        talk('I arrived')
        return 'succ'

class Goto_d_to(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
        
    def execute(self,userdata):
        rospy.loginfo('STATE : GOTO_AREA')
        self.tries+=1
        if self.tries==5:
            return 'tries'
        tf_man.pub_static_tf(pos = [0.0,-2.0,0.0],point_name='target_0', ref='base_link')
        rospy.sleep(0.5)
        tf_man.change_ref_frame_tf(point_name='target_0')
        rospy.sleep(0.5)
        pos,_=tf_man.getTF(target_frame='target_0')
        if type(pos[0]) is not bool:
            gaze.absolute(*pos)
        # base.move_d_to(target_distance = 0.5, target_link='target')
        talk('I arrived')
        return 'succ'

def init(node_name):
    global head, arm, line_detector, goal, navclient, base, tf_man, gaze
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('SMACH_wait_door_go_area')

    head = moveit_commander.MoveGroupCommander('head')
    arm =  moveit_commander.MoveGroupCommander('arm')
    tf_man = TF_MANAGER()
    gaze = GAZE()
    line_detector=LineDetector()
    base = OMNIBASE()
#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("SMACH_wait_door_go_area")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"

    with sm:
        smach.StateMachine.add("INITIAL",   Initial(),      transitions = {'failed':'INITIAL',  'succ':'WAIT_DOOR', 'tries':'INITIAL'}) 
        smach.StateMachine.add("WAIT_DOOR", Wait_door(),    transitions = {'failed':'WAIT_DOOR','succ':'GOTO_AREA','tries':'WAIT_DOOR'}) 
        smach.StateMachine.add("GOTO_AREA", Goto_area(),    transitions = {'failed':'GOTO_AREA','succ':'GOTO_d_to',      'tries':'GOTO_AREA'}) 
        smach.StateMachine.add("GOTO_d_to", Goto_d_to(),    transitions = {'failed':'GOTO_d_to','succ':'END',      'tries':'GOTO_d_to'}) 
    outcome = sm.execute()