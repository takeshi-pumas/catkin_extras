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
            
        #head.set_named_target('neutral')
        #brazo.set_named_target('go')
        rospy.sleep(0.8)
        return 'succ'

# --------------------------------------------------
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

class Wait_instruction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goto', 'findPerson', 'findObject', 'bring', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        global location, obj    , person
        person=''
        rospy.loginfo('STATE : Wait for instruction')
        #res = omni_base.move_base(known_location='inspection_point')
        #if res != 3:
        #    return 'tries'
        print('Waiting for instruction')
        talk('Waiting for instruction')
        rospy.sleep(2.0)
        try:
            res = rospy.wait_for_message( '/recognizedSpeech',RecognizedSpeech, timeout = 5.0)
            print ("POCKERT",res.hypothesis[0])
            location=res.hypothesis[0].split(' ')[-1]
            #print ('loc',location)
            
        except Exception:   #answer = res2
            return 'tries'
       
        if (res.hypothesis[0].split(' ')[0]=='go'):
            print ('go')
            location=res.hypothesis[0].split(' ')[-1]
            if location=='room':
                location_room= res.hypothesis[0].split(' ')[-2]+'_room'
                print(location_room )
                location=location_room
                print (location,'###')

            print ('transition to go')
        
            return 'goto'

        if (res.hypothesis[0].split(' ')[1]=='Michael') or (res.hypothesis[0].split(' ')[1]=='Morgan'):

            
        else: 
            talk('I did not understand')
            return 'tries'
            
# --------------------------------------------------

class Goto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : GOTO')
        print('Robot goto command')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        print (location)
        if self.tries == 3:
            return 'tries'
        res = omni_base.move_base(known_location=location)
        talk(f'I already arrived to {location}')
        if res == 3:
            return 'succ'
        else:
            return 'failed'

class findPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : GOTO')
        print('Robot goto command')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        #res = omni_base.move_base(known_location=location)
        talk(f'I already arrived to {location}')
        '''if res == 3:
            return 'succ'
        else:
            return 'failed'''

        head.set_joint_values([0.0, 0.3])
        talk(f'I am looking for {person}, look at me, please')
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'

        res, _ = wait_for_face()  # default 10 secs
        rospy.sleep(0.7)
        print (res)

        print('Checking for faces')
        if res != None:
            #return 0

            name = res.Ids.ids

            print('RESPONSE', name)
            if name != person:
                print('No face Found, Keep scanning')
                talk('I can not see you , I will try again')
                return 'failed'
            else:
                talk(f'I found you {person}')
        else:
            return 'failed'

class findObject(smach.State):
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
        
        res = omni_base.move_base(known_location=location)
        talk(f'I already arrived to {location}')
        if res == 3:
            head.set_named_target('left')
            rospy.sleep(1.0)
            head.set_named_target('right')
            rospy.sleep(1.0)
            talk(f'Sorry I didnt find a {obj}')
            return 'succ'
        else:
            return 'failed'

class Bring(smach.State):
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
        
        res = omni_base.move_base(known_location=location)
        talk(f'I already arrived to {location}')
        if res == 3:
            head.set_named_target('left')
            rospy.sleep(1.0)
            head.set_named_target('right')
            rospy.sleep(1.0)
            talk(f'Sorry I could not bring a {obj}')
            return 'succ'
        else:
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
    sis = smach_ros.IntrospectionServer(
        'SMACH_VIEW_SERVER', sm, '/SM_GPSR')
    sis.start()

    with sm:
        # State machine for Restaurant

        smach.StateMachine.add("INITIAL",           Initial(),          transitions={'failed': 'INITIAL',       'succ': 'WAIT_DOOR',   'tries': 'END'})
        smach.StateMachine.add("WAIT_DOOR",    Wait_door(),   transitions={'failed': 'WAIT_DOOR','succ': 'WAIT_INSTRUCTION',        'tries': 'WAIT_DOOR'})
        smach.StateMachine.add("WAIT_INSTRUCTION",         Wait_instruction(),        transitions={'goto': 'GOTO',     'findPerson': 'FIND_PERSON',      'findObject': 'FIND_OBJECT','bring':'BRING','tries': 'WAIT_INSTRUCTION'})

        #smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions={'failed': 'WAIT_PUSH_HAND','succ': 'WAIT_INSTRUCTION',        'tries': 'INITIAL'})
        #smach.StateMachine.add("WAIT_DOOR_OPENED",  Wait_door_opened(),   transitions={'failed': 'WAIT_DOOR_OPENED','succ': 'GOTO_DOOR',        'tries': 'INITIAL'})
        
        smach.StateMachine.add("GOTO",          Goto(),         transitions={'failed': 'GOTO',      'succ': 'WAIT_INSTRUCTION','tries': 'GOTO'})
        smach.StateMachine.add("FIND_PERSON",         findPerson(),        transitions={'failed': 'FIND_PERSON',     'succ': 'FIND_PERSON',        'tries': 'FIND_PERSON'})
        smach.StateMachine.add("FIND_OBJECT",         findObject(),        transitions={'failed': 'FIND_PERSON',     'succ': 'FIND_PERSON',        'tries': 'FIND_PERSON'})
        smach.StateMachine.add("BRING",         Bring(),        transitions={'failed': 'FIND_PERSON',     'succ': 'FIND_PERSON',        'tries': 'FIND_PERSON'})
        

        #smach.StateMachine.add("GOTO_FACE",         Goto_face(),        transitions={'failed': 'GOTO_FACE',     'succ': 'LEAD_TO_LIVING_ROOM',   'tries': 'SCAN_FACE'})
        #smach.StateMachine.add("LEAD_TO_LIVING_ROOM",Lead_to_living_room(), transitions={'failed': 'LEAD_TO_LIVING_ROOM','succ': 'FIND_SITTING_PLACE','tries': 'END'})
        #smach.StateMachine.add("FIND_SITTING_PLACE", Find_sitting_place(),  transitions={'failed': 'FIND_SITTING_PLACE','succ': 'INTRODUCE_GUEST',    'tries': 'END'})
        #smach.StateMachine.add("INTRODUCE_GUEST",   Introduce_guest(),      transitions={'failed':'INTRODUCE_GUEST','succ':'WAIT_PUSH_HAND',    'tries':'WAIT_PUSH_HAND'})

    outcome = sm.execute()
