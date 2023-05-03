#!/usr/bin/env python3

from smach_utils2 import *

##### Define state INITIAL #####

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
        #clean_knowledge()
        head.set_named_target('neutral')
        #print('head listo')
        #brazo.set_named_target('go')
        #print('brazo listo')
        rospy.sleep(0.8)

        return 'succ'

#-------------------------------------------------

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
            talk('I am waiting for door to be opened')
            rospy.sleep(0.7)

        if not line_detector.line_found():
            rospy.sleep(0.7)
            talk('I can see the door is opened, entering')
            rospy.sleep(0.7)
            return 'succ'
        else:
            return 'tries'

# --------------------------------------------------


class Enter_arena(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('I am entering to, arena')
        #res = omni_base.move_base(known_location='door')
        #print(res)
        rospy.sleep(8.0)
        return 'succ'
        if res == 3:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

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
        talk('Gently... push my hand to continue')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'
#########################################################################################################
class Goto_inspection_point(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, inspection point')
        #print ('navigating to room'+str(self.next_room))
        inspection_point = 'inspection'
        res = omni_base.move_base(known_location=inspection_point)
        print(res)

        if res == 3:
            talk('I am ready for robot inspection')
            return 'succ'

        else:
            talk('Navigation Failed, retrying')
            return 'failed'


# --------------------------------------------------


class Goto_exit(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, exit')
        #print ('navigating to room'+str(self.next_room))
        res = omni_base.move_base(known_location='exit')
        print(res)

        if res == 3:
            talk('test done! thanks for your attenntion!')
            return 'succ'

        else:
            talk('Navigation Failed, retrying')
            return 'failed'


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
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_ROBOT_INSPECTION')
    sis.start()

    with sm:
        # State machine for Restaurant
        smach.StateMachine.add("INITIAL",       Initial(),      transitions={'failed': 'INITIAL',       'succ': 'WAIT_DOOR',       'tries': 'INITIAL'})
        #################################################################################################################################################################
        smach.StateMachine.add("WAIT_DOOR",     Wait_door(),    transitions={'failed': 'WAIT_DOOR',     'succ': 'ENTER_ARENA',      'tries': 'WAIT_DOOR'})
        smach.StateMachine.add("ENTER_ARENA",   Enter_arena(),  transitions={'failed': 'ENTER_ARENA',   'succ': 'GOTO_INSPECTION_POINT',   'tries': 'ENTER_ARENA'})
        #################################################################################################################################################################
        smach.StateMachine.add("GOTO_INSPECTION_POINT",     Goto_inspection_point(),       transitions={'failed': 'GOTO_INSPECTION_POINT',    'succ': 'WAIT_HAND', 'tries': 'GOTO_INSPECTION_POINT'})
        smach.StateMachine.add("WAIT_HAND",     Wait_push_hand(),       transitions={'failed': 'WAIT_HAND',    'succ': 'GOTO_EXIT', 'tries': 'WAIT_HAND'})
        
        smach.StateMachine.add("GOTO_EXIT",     Goto_exit(),     transitions={'failed': 'END','succ': 'END'    , 'tries': 'END'})
        ##################################################################################################################################################################
        


    outcome = sm.execute()
