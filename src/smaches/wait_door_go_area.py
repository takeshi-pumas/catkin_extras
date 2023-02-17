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
import actionlib
from hmm_navigation.msg import NavigateActionGoal, NavigateAction


##### Define state INITIAL #####
navclient = actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB

def move_base(goal_x=0.0,goal_y=0.0,goal_yaw=0.0,time_out=10, known_location='None'):
    #Create and fill Navigate Action Goal message
    nav_goal = NavigateActionGoal()
    nav_goal.goal.x = goal_x
    nav_goal.goal.y = goal_y
    nav_goal.goal.yaw = goal_yaw
    nav_goal.goal.timeout = time_out
    nav_goal.goal.known_location = known_location
    print (nav_goal)

    # send message to the action server
    navclient.send_goal(nav_goal.goal)

    # wait for the action server to complete the order
    navclient.wait_for_result(timeout=rospy.Duration(time_out))

    # Results of navigation
    # PENDING         = 0   The goal has yet to be processed by the action server
    # ACTIVE          = 1   The goal is currently being processed by the action server
    # PREEMPTED       = 2   The goal received a cancel request after it started executing
                            # and has since completed its execution (Terminal State)
    # SUCCEEDED       = 3   The goal was achieved successfully by the action server (Terminal State)
    # ABORTED         = 4   The goal was aborted during execution by the action server due
                            # to some failure (Terminal State)
    # REJECTED        = 5   The goal was rejected by the action server without being processed,
                            # because the goal was unattainable or invalid (Terminal State)
    # PREEMPTING      = 6   The goal received a cancel request after it started executing
                            # and has not yet completed execution
    # RECALLING       = 7   The goal received a cancel request before it started executing,
                                # but the action server has not yet confirmed that the goal is canceled
    # RECALLED        = 8   The goal received a cancel request before it started executing
                            # and was successfully cancelled (Terminal State)
    # LOST            = 9   An action client can determine that a goal is LOST. This should not be
                            # sent over the wire by an action server
    action_state = navclient.get_state()
    return navclient.get_state()

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
        head.set_joint_value_target([0.0,-0.5])
        head.go()
        rospy.sleep(1.0)
        if self.tries==5:
            return 'tries'
        move_base(known_location='living_room')
        #talk('I arrived')
        return 'succ'
    
def init(node_name):
    global head, arm, line_detector, goal, navclient
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('SMACH_wait_door_go_area')

    head = moveit_commander.MoveGroupCommander('head')
    arm =  moveit_commander.MoveGroupCommander('arm')
    line_detector=LineDetector()
#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("SMACH_wait_door_go_area")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"

    with sm:
        smach.StateMachine.add("INITIAL",   Initial(),      transitions = {'failed':'INITIAL',  'succ':'WAIT_DOOR', 'tries':'INITIAL'}) 
        smach.StateMachine.add("WAIT_DOOR", Wait_door(),    transitions = {'failed':'WAIT_DOOR','succ':'GOTO_AREA','tries':'WAIT_DOOR'}) 
        smach.StateMachine.add("GOTO_AREA", Goto_area(),    transitions = {'failed':'GOTO_AREA','succ':'END',      'tries':'GOTO_AREA'}) 
    outcome = sm.execute()

 
    
