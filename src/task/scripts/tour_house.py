#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String



class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        return 'succ'
        
        if self.tries == 3:
            return 'tries'        
             
              

# --------------------------------------------------
def init(node_name):
       print('smach ready')
   # --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),          transitions={'failed': 'INITIAL',           
                                                                                         #'succ': 'SCAN_SHELF',   
                                                                                         'succ': 'END',   
                                                                                         'tries': 'END'})
        
        


    outcome = sm.execute()
