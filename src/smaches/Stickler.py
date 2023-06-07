#!/usr/bin/env python3

from smach_utils2 import *
import torch
import clip
from PIL import Image
################################################(TO UTILS?)
global model , preprocess
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)
    
##### Define state INITIAL #####

# --------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        global rooms
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        clean_knowledge()
        ##############

        ##########
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

        if res:
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

        if self.tries==1:head.set_joint_values([ 0.0, 0.0])
        if self.tries==2:head.set_joint_values([ 0.3, 0.1])
        if self.tries==3:head.set_joint_values([-0.3, 0.1])

        
        rospy.sleep(0.7)
        humanpose=detect_human_to_tf()

        print('Detecting Humans ')


        if humanpose== False:
            print ('no human ')
            return 'failed'
        else : 
            talk('Human Found, Getting close ')
            head.to_tf('human')
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
        if self.tries == 5:
            self.tries=0
            #return 'tries'
        elif self.tries == 1: 
            talk('Navigating to bedroom')
            next_room='bedroom'
        elif self.tries == 2: 
                    talk('Navigating to bedroom')
                    next_room='bedroom'
        elif self.tries == 3: 
                    talk('Navigating to dining room')
                    next_room='dining_room'
        elif self.tries == 4: 
                    talk('Navigating to kitchen')
                    next_room='kitchen'

       
       

        res = omni_base.move_base(known_location=next_room)
        print(res)

        if res :


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
        res = omni_base.move_d_to(1.0,'human')

        head.to_tf('human')

        print (res)
        return 'succ'

        if res == 3:
            print ( "is he drinking?")
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

        rospy.sleep(0.9)
        ##### SHOES NO SHOES DETECTOR
        img=rgbd.get_image()
        print ('got image for feet analysis')
        keys=[ "feet", "shoes",'socks','sock','sandals']
        image = preprocess(Image.fromarray(img)).unsqueeze(0).to(device) #img array from grbd get image
        text = clip.tokenize(keys).to(device)

        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)
            
            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        print("Label probs:", probs,keys[np.argmax(probs)] , probs,keys)  # prints: [[0.9927937  0.00421068 0.00299572]]
        if keys[np.argmax(probs)] in ['sandals','feet','socks']:
            talk ('Thanks for not wearing shoes.... Rule 1 is observed')
            print ('Not wearing shoes')
        else: 
            talk('Rule number 1... no shoes please...')
            ##
            return 'succ'


    
        
    
###########################################################################################################

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
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STICKLER')
    sis.start()

    with sm:
        # State machine for Restaurant
        #################################################################################################################################################################
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',       'succ': 'WAIT_PUSH_HAND',   'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND','succ': 'FIND_HUMAN',        'tries': 'END'})
        smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',    'succ': 'GOTO_HUMAN'    , 'tries': 'GOTO_NEXT_ROOM'})
        smach.StateMachine.add("GOTO_HUMAN",         Goto_human(),          transitions={'failed': 'GOTO_HUMAN',    'succ': 'ANALYZE_HUMAN' , 'tries': 'FIND_HUMAN'})
        smach.StateMachine.add("ANALYZE_HUMAN",      Analyze_human(),       transitions={'failed': 'FIND_HUMAN',    'succ': 'GOTO_NEXT_ROOM'})
        smach.StateMachine.add("GOTO_NEXT_ROOM",     Goto_next_room(),      transitions={'failed': 'GOTO_NEXT_ROOM','succ': 'FIND_HUMAN'    , 'tries': 'FIND_HUMAN'})
        #################################################################################################################################################################
        


    outcome = sm.execute()
