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
#######################################
class Choose_room(smach.State):
    def __init__(self):
        smach.State.__init__(self,output_keys=['room'], outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):       
        self.tries+=1 
        rospy.loginfo('STATE : CHOOSE_ROOM')
        
        known_locs=yaml_to_df("/known_locations_storingTMR.yaml")
        print('known_locs')
        print(known_locs)

        random_choice = np.random.choice(known_locs['child_id_frame'])
        print (f'random destination is {random_choice}')
        userdata.room=random_choice
        return 'succ'
        
        if self.tries == 5:
            userdata.room='home'

            return 'succ'        
#####################################
class Goto_room(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=['room'], outcomes=['succ', 'failed', 'end'])
    def execute(self, userdata):        
        rospy.loginfo('STATE : GOTO_ROOM')
        print (f'userdata.room{userdata.room}')
        res = omni_base.move_base(known_location=userdata.room, time_out=40)
        print (f'res{res}')
        return 'succ'
        if userdata.room == 'home':
            return 'end'        
             
              

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
                                                                                         'succ': 'CHOOSE_ROOM',   
                                                                                         'tries': 'END'})
        
        smach.StateMachine.add("CHOOSE_ROOM",           Choose_room(),          transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'GOTO_ROOM'})
        smach.StateMachine.add("GOTO_ROOM",           Goto_room(),          transitions={'failed': 'GOTO_ROOM',           
                                                                                         'succ': 'CHOOSE_ROOM',   
                                                                                         'end': 'END'})


    outcome = sm.execute()
