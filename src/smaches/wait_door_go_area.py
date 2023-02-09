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
# import tf2_ros as tf2
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from grasp_utils import *
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
        rospy.sleep(0.5)
        if not line_detector.line_found():
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
        msg = String()
        msg.data = 'dining_room' #Any known location
        goal.publish(msg)
        # print (result_state)
        if self.tries==5:
            self.tries=0
            return 'tries'
        #rospy.publisher('goal_location', std_msgs/String)
        
        return 'succ'
    
def init(node_name):
    global head, arm, line_detector, goal
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('SMACH_wait_door_go_area')

    head = moveit_commander.MoveGroupCommander('head')
    # wb = moveit_commander.MoveGroupCommander('whole_body')
    arm =  moveit_commander.MoveGroupCommander('arm')

    goal = rospy.Publisher('/goal_location', String, queue_size=10)
    line_detector=LineDetector()
    
    
    
#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("SMACH_wait_door_go_area")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    
    #sm.userdata.clear = False
    # sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_MOVE_SCAN_GRASP')
    # sis.start()


    with sm:
        smach.StateMachine.add("INITIAL",   Initial(),      transitions = {'failed':'INITIAL',  'succ':'WAIT_DOOR', 'tries':'INITIAL'}) 
        smach.StateMachine.add("WAIT_DOOR", Wait_door(),    transitions = {'failed':'WAIT_DOOR','succ':'GOTO_AREA','tries':'WAIT_DOOR'}) 
        smach.StateMachine.add("GOTO_AREA", Goto_area(),    transitions = {'failed':'GOTO_AREA','succ':'END',      'tries':'GOTO_AREA'}) 
    outcome = sm.execute()

 
    
