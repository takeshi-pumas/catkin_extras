#!/usr/bin/env python3
from common.states import WaitPushHand, WaitDoorOpen
from common.utils import Talker
import rospy
import smach


if __name__ == '__main__':
    print("Testing Common states")
    rospy.init_node("test_common_states")
    voice = Talker()
    sm = smach.StateMachine(outcomes=['succ', 'failed'])
    # voice.talk("Testing common states")
    with sm:
        smach.StateMachine.add('WAIT_PUSH_HAND', 
        WaitPushHand(talker = voice, timeout=1000), transitions={'succ': 'WAIT_DOOR_OPEN', 'failed': 'WAIT_PUSH_HAND'})
        smach.StateMachine.add('WAIT_DOOR_OPEN', 
        WaitDoorOpen(talker = voice, timeout=1000), transitions={'succ': 'succ', 'failed': 'WAIT_DOOR_OPEN'})
    outcome = sm.execute()