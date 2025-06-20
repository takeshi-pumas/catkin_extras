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

        global forbiden_room , xys , room_names 
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        
        #READ YAML ROOMS XYS
        df=yaml_to_df()
        xys=[]
        xys.append(df[df['child_id_frame']=='bedroom'][['x','y']].values.ravel())
        xys.append(df[df['child_id_frame']=='living_room'][['x','y']].values.ravel())
        xys.append(df[df['child_id_frame']=='dining_room'][['x','y']].values.ravel())
        xys.append(df[df['child_id_frame']=='kitchen'][['x','y']].values.ravel())
        room_names=['bedroom','living_room','dining_room','kitchen']
        xys, room_names
        #####
        ####FORBIDEN ROOM 
        forbiden_room='dining_room'

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
            self.tries=0
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
        
        
        
        if self.tries   <= 1: 
            talk('Navigating to  living room')
            next_room='living_room'
        elif self.tries == 2: 
                    talk('Navigating to dining room')
                    next_room='dining_room'
        elif self.tries == 3: 
                    talk('Navigating to kitchen')
                    next_room='kitchen'
        elif self.tries == 4: 
                    talk('Navigating to bedroom')
                    next_room='bedroom'
                    self.tries=0
       

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
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries','forbidden'])
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
        
        human_pos,_=tf_man.getTF('human')
        print   (human_pos[:2])
        human_xy=np.asarray(human_pos[:2])

        dists=(human_xy-np.asarray(xys))
        
        

        if room_names[np.linalg.norm(dists, axis=1).argmin()]== forbiden_room:
            head.to_tf('human')
            talk (f'human found in{room_names[np.linalg.norm(dists, axis=1).argmin()]} which is forbidden, please follow me to an allowed room')
            return 'forbidden'


        
        

        if res :
            self.tries=0
            return 'succ'
            
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

###########################################################################################################


class Lead_to_living_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        _,guest_name = get_waiting_guests()
        talk(f'{guest_name}... I will lead you to the living room, please follow me')
        # talk('Navigating to ,living room')
        res = omni_base.move_base(known_location='living_room')
        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

    
        
    
###########################################################################################################
class Analyze_human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Analyze area close to human')

       
        
        head.to_tf('human')
        ###image open pose tocayo magic.... #DRINKS
        

        


        human_pos,_=tf_man.getTF('human')
        
        head.absolute(human_pos[0],human_pos[1],0.1)
        rospy.sleep(2.9)
        ##### SHOES NO SHOES DETECTOR
        img=rgbd.get_image()
        cv2.imwrite('feet.png',img)
        print ('got image for feet analysis')
        keys=[ "feet", "shoes",'socks','sock','sandals']
        image = preprocess(Image.fromarray(img)).unsqueeze(0).to(device) #img array from grbd get image
        text = clip.tokenize(keys).to(device)

        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)
            
            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        print("Label probs:", probs,keys[np.argmax(probs)] ,keys, probs[0][1] ) # prints: [[0.9927937  0.00421068 0.00299572]]
        if (keys[np.argmax(probs)] in ['sandals','feet','socks', 'sock' ]   ) or ( probs[0][1]  < 0.36) :  

            talk ('Thanks for not wearing shoes.... Rule 1 is observed')
            print ('Not wearing shoes')
        else: 
            talk('Rule number 1... no shoes please... Would you mind taking them off?.... thank you ')
            ##
        return 'succ'


    
#########################################################################################################
class Analyze_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : Analyze_human')

        
        self.tries+=1
        #NO MOVEIT
        #hv=head_mvit.get_current_joint_values()
        #hv[0]=1.0
        #head_mvit.go(hv)

        
        
        if self.tries==1: head.set_joint_values([0.9,-1])
        elif self.tries==2: head.set_joint_values([-0.9,-1])
        elif self.tries==3:
            self.tries=0
            return 'tries'
        rospy.sleep(2.9)
        ##### Segment and analyze
        img=rgbd.get_image()
        cv2.imwrite('rubb.png',img)
        print ('got image for segmentation')
        res=segmentation_server.call()
        origin_map_img=[round(contoured.shape[0]*0.5) ,round(contoured.shape[1]*0.5)]

        if len(res.poses.data)==0:
            talk('Rule no littering Observed. .. no Trash  in area next to human....')
            return 'failed'

        else:
            print('object found')
            
            poses=np.asarray(res.poses.data)
            poses=poses.reshape((int(len(poses)/3) ,3     )      )  
            
            for i,pose in enumerate(poses):
                #print (f'Occupancy map at point object {i}->', contoured[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                point_name=f'object_{i}'
            
                if contoured[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                    print ('reject point, most likely part of arena, occupied inflated map')
                else:

                    tf_man.pub_static_tf(pos=pose, point_name=point_name, ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                    rospy.sleep(0.3)
                    tf_man.change_ref_frame_tf(point_name=point_name, new_frame='map')
                    rospy.sleep(0.3)
                    print (f"object found at map coords.{pose} ")
                    head.to_tf('human')
                    rospy.sleep(1.9)
                    talk ('Rule number 2 Broken. Rubbish detected Would you mind picking it up')
                    self.tries=0
                    return 'succ'

                talk('Rule no littering Observed. .. no Trash  in area next to human....')
                self.tries=0
                return 'failed'
                    #### TODO  CHECK FOR COMPLIANCE OF RULE

                
        


        #res=segmentation_server.call()
        #if len(res.poses.data)==0: 
        #    talk('Rule no littering Observed. .. no Trash  in area next to human....')
        #    return 'failed'
        #else:
        #    print('object found')
        #    talk ('Something detected. Maybe trash Rule broken. NO littering')
        #    poses=np.asarray(res.poses.data)
        #    ### TFS FOR OBJECTS using segmentation server
        #    poses=poses.reshape((int(len(poses)/3) ,3     )      )  
        #    print (poses.shape)
        #    for i,pose in enumerate(poses):
        #        point_name=f'object_{i}'
        #        print (point_name)
        #        tf_man.pub_static_tf(pos=pose, point_name=point_name, ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
        #        rospy.sleep(0.3)
        #        tf_man.change_ref_frame_tf(point_name=point_name, new_frame='map')
        #        rospy.sleep(0.3)
        #    #####################


            
        



"""


        keys=[ "trash", "paper",'rubbish', 'floor']
        image = preprocess(Image.fromarray(img)).unsqueeze(0).to(device) #img array from grbd get image
        text = clip.tokenize(keys).to(device)

        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)
            
            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        print("Label probs:", probs,keys[np.argmax(probs)] , probs)  # prints: [[0.9927937  0.00421068 0.00299572]]
        if keys[np.argmax(probs)]  in ['trash ','paper', 'rubbish']:
            talk ('Something detected. Myabe trash?')
            print ('Not wearing shoes')
        else: 
            talk('Rule number 2 Observed. .. no Trash next to human.... thank you ')

        return 'succ'
        """
    
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
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',       'succ': 'WAIT_PUSH_HAND',   'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND','succ': 'FIND_HUMAN',        'tries': 'END'})
        smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',    'succ': 'GOTO_HUMAN'    , 'tries': 'GOTO_NEXT_ROOM'})
        smach.StateMachine.add("GOTO_HUMAN",         Goto_human(),          transitions={'failed': 'GOTO_HUMAN',    'succ': 'ANALYZE_HUMAN' , 'tries': 'FIND_HUMAN' , 'forbidden':'LEAD_TO_LIVING_ROOM'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM",Lead_to_living_room(), transitions={'failed': 'LEAD_TO_LIVING_ROOM','succ': 'GOTO_NEXT_ROOM','tries': 'END'})
        smach.StateMachine.add("ANALYZE_HUMAN",      Analyze_human(),       transitions={'failed': 'FIND_HUMAN',    'succ': 'ANALYZE_AREA'})
        smach.StateMachine.add("ANALYZE_AREA",      Analyze_area(),         transitions={'failed': 'ANALYZE_AREA' , 'succ': 'GOTO_NEXT_ROOM' , 'tries': 'GOTO_NEXT_ROOM'})   #
        smach.StateMachine.add("GOTO_NEXT_ROOM",     Goto_next_room(),      transitions={'failed': 'GOTO_NEXT_ROOM','succ': 'FIND_HUMAN'    , 'tries': 'FIND_HUMAN'})
        #################################################################################################################################################################
        #################################################################################################################################################################
        ##################################################################################################################################################################
        


    outcome = sm.execute()
