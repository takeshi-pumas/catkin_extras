#!/usr/bin/env python3
import rospy
import smach
from smach_ros import SimpleActionState
from object_classification.srv import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray
import numpy as np 

from utils.src.utils import misc_utils, grasp_utils, nav_utils

rospy.init_node('Serve_Breakfast_SMACH')
rgbd = misc_utils.RGBD()
head = grasp_utils.GAZE()
omni_base = nav_utils.NAVIGATION()
voice = misc_utils.TALKER()
brazo = grasp_utils.ARM()
wrist = grasp_utils.WRIST_SENSOR()
classify_client = rospy.ServiceProxy('/classify', Classify)

bridge = CvBridge()

def wait_for_push_hand(time=10):

    start_time = rospy.get_time()
    time= 10
    print('timeout will be ',time,'seconds')
    while rospy.get_time() - start_time < time:
        torque = wrist.get_torque()
        if np.abs(torque[1])>1.0:
            print(' Hand Pused Ready To Start')
            #takeshi_talk_pub.publish(string_to_Voice())
            #talk('Im ready to start')
            return True

    if (rospy.get_time() - start_time >= time):
        print(time, 'secs have elapsed with no hand push')
        return False

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['target_pose'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        rospy.loginfo('STATE : INITIAL')
        rospy.loginfo(f'Try {self.tries} of 5 attempts')
        #userdata.goal = GraspAction()
        target_pose = Float32MultiArray()
        target_pose.data = [1.0, 1.5, 0.3]
        userdata.target_pose = target_pose
        return 'succ'


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
        voice.talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'
        
class Goto_kitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succ', 'failed'])
        self.tries = 0
    
    def execute(self, userdata):
        rospy.loginfo("SMACH : Go to kitchen")
        self.tries += 1
        res = omni_base.move_base(known_location = 'kitchen')
        print(res)

        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation Failed, retrying')
            return 'failed'
        
class Goto_kitchen_container(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succ', 'failed'])
        self.tries = 0
    
    def execute(self, userdata):
        rospy.loginfo("SMACH: Go to breakfast container")
        self.tries += 1
        res = omni_base.move_base(known_location = 'breakfast_container')
        print(res)

        if res:
            self.tries = 0
            return 'succ'
        else:
            voice.talk('Navigation failed, retrying')
            return 'failed'
        
class Scan_container(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             output_keys=['obj_detected'],
                             outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo("SMACH: Scan breakfast container")
        self.tries += 1
        head.set_joint_values([0.0,-0.65])


        rospy.sleep(0.6)
        image= cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR)
        img_msg  = bridge.cv2_to_imgmsg(image)
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)

        userdata.obj_detected = res.names
        
        print(res)

        # for i  in range(len(res.names)):
        #     pass
        return 'succ'

class Decide_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             input_keys=['obj_detected'],
                             output_keys=['obj_to_grasp'],
                             outcomes = ['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo("SMACH : Decide which object is better to grasp")
        self.tries += 1
        print(userdata.obj_detected)
        head.set_named_target('neutral')
        userdata.obj_to_grasp = []



if __name__ == '__main__':

    sm = smach.StateMachine(outcomes=['success', 'failure'])#, input_keys=['target_pose'])
    # sm.userdata.target_pose = []
    with sm:
        smach.StateMachine.add("INITIAL", Initial(),              
                        transitions={'failed': 'INITIAL', 'succ': 'GRASP_GOAL'})
        smach.StateMachine.add("WAIT_PUSH_HAND", Wait_push_hand(),              
                        transitions={'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND'})
        smach.StateMachine.add("GOTO_KITCHEN", Goto_kitchen(),              
                        transitions={'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND'})
        smach.StateMachine.add("GOTO_KITCHEN_CONTAINER", Goto_kitchen_container(),              
                        transitions={'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND'})
        smach.StateMachine.add("SCAN_CONTAINER", Scan_container(),              
                        transitions={'failed': 'INITIAL', 'succ': 'WAIT_PUSH_HAND'})
        
        smach.StateMachine.add("DECIDE_GRASP", Decide_grasp(),              
                        transitions={'failed': 'INITIAL', 'succ': 'GRASP_GOAL'})
        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction, goal_slots=['target_pose']),              
                        transitions={'preempted': 'INITIAL', 'succeeded': 'INITIAL', 'aborted': 'INITIAL'})
    
    outcome = sm.execute()

