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

#########################################################################################################
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        global forbiden_room,closest_room , xys , room_names 
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
        room_names=['bedroom','living_room','dining_room','kitchen']4
        xys, room_names
        #####
        ####FORBIDEN ROOM 
        forbiden_room='bedroom'
        closest_room='kitchen'

        head.set_named_target('neutral')
        #print('head listo')
        #brazo.set_named_target('go')
        #print('brazo listo')
        rospy.sleep(0.8)

        return 'succ'


#########################################################################################################
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


#########################################################################################################
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


#########################################################################################################
class Find_human(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries','forbidden'])
        self.tries = 0

    def execute(self, userdata):
        

        rospy.loginfo('State : Find_human')
        talk('Scanning the room for humans')
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'

        if self.tries==1:head.set_joint_values([ 0.0, 0.0])
        if self.tries==2:head.set_joint_values([ 0.5, 0.1])
        if self.tries==3:head.set_joint_values([-0.5, 0.1])
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]
        humanpose=detect_human_to_tf()  #make sure service is running
        if humanpose== False:
            print ('no human ')
            return 'failed'
            
##############################################################
        else : 
            human_pose,_=tf_man.getTF('human')
            #pose=human_pose[:2]
            #dists=(pose-np.asarray(xys))
            #human_room=room_names[np.linalg.norm(dists, axis=1).argmin()]
            #print(f'human in {human_room}')
            #robot_pose=get_robot_px()
            #dists=(robot_pose-np.asarray(xys))
            #robot_room=room_names[np.linalg.norm(dists, axis=1).argmin()]
            #print(f'Robot  in {robot_room}')
            

            ###################

            living_room_px_region=np.asarray(((1012, 1045), (1125, 1151)))
            kitchen_px_region=np.asarray(((1123, 1040), (1217, 1149)))
            bedroom_px_region=np.asarray(((1123, 971), (1212, 1038)))
            dining_room_px_region=np.asarray(((1027, 968), (1122, 1043)))

            pose=human_pose[:2]
            px_pose_human=np.asarray(([origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]))
            room_human =check_room_px(np.flip(px_pose_human),living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region)
            pose=get_robot_px()
            px_pose_robot=np.asarray((origin_map_img[1]+pose[1],origin_map_img[0]+pose[0]))
        
            room_robot = check_room_px(np.flip(px_pose_robot),living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region)
            

            
            print('room_robot,room_human',room_robot,room_human)
            print ('px human',px_pose_human)
            print('room_human',room_human)

            #########################


            #if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                    #print ('reject point, most likely part of the audience, outside of the arena map')
                    #talk ('Human is close to map edge, maybe a part of the audience')
                    #rospy.sleep(1.0)
                    #return 'failed'
            
            if room_robot != room_human: 
                print('maybe false positive... ignoring... ')
                #talk ('Human is not in the same room as me... Igonring ')
                #rospy.sleep(1.0)
                return 'failed'
            
            if room_human==forbiden_room:
                head.to_tf('human')
                talk (f'human found in forbidden room {room_human} ')
                rospy.sleep(0.6)
                talk('I will take him to a valid location')
                self.tries=0
                return 'forbidden'
                
            talk('Human Found')
            self.tries=0
            return 'succ'    


class Goto_next_room(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.next_room=1

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

       
        self.tries += 1
        
        
        
        if self.tries   <= 1: 
                    talk('Navigating to bed room')
                    next_room='bedroom'
            
        elif self.tries == 2: 
                    talk('Navigating to kitchen')
                    next_room='kitchen'
        elif self.tries == 3: 
                    talk('Navigating to  living room')
                    next_room='living_room'
            
        elif self.tries == 4: 
                    talk('Navigating to dining room')
                    next_room='dining_room'
                    self.tries=0
       

        res = omni_base.move_base(known_location=next_room)
        print(res)

        if res :


            return 'succ'

        else:
            talk('Navigation Failed, retrying')
            self.tries +=-1
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
        
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]
        

        human_pose,_=tf_man.getTF('human')
        
        
        living_room_px_region=np.asarray(((1012, 1045), (1125, 1151)))
        kitchen_px_region=np.asarray(((1123, 1040), (1217, 1149)))
        bedroom_px_region=np.asarray(((1123, 971), (1212, 1038)))
        dining_room_px_region=np.asarray(((1027, 968), (1122, 1043)))

        pose=human_pose[:2]
        px_pose_human=np.asarray(([origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]))
        room_human =check_room_px(np.flip(px_pose_human),living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region)
        if room_human== forbiden_room:
        #if room_names[np.linalg.norm(dists, axis=1).argmin()]== forbiden_room:
            head.to_tf('human')
            talk (f' Sorry  no one is allowed in the {forbiden_room} ')
            rospy.sleep(0.6)
            
            return 'forbidden'

        print('getting close to human')
        head.to_tf('human')

        res = omni_base.move_d_to(0.65,'human')

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




        head.to_tf('human')
        rospy.sleep(0.5)
            
        head.set_joint_values([0,1.3])
        rospy.sleep(0.5)
        res = omni_base.move_d_to(1.0,'human')


        talk(f'{guest_name}... I will lead you to the {closest_room}, please follow me')
        # talk('Navigating to ,living room')
        res = omni_base.move_base(known_location=closest_room)
        if res:
            self.tries=0
            talk( ' You can remain here, thank you')
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

        rospy.loginfo('STATE : Analyze  human')

        head.set_joint_values([0,1.3])
        rospy.sleep(1.0)
            
        
        human_pos,_=tf_man.getTF('human')
        #head.absolute(human_pos[0],human_pos[1],0.1)
        ##### SHOES NO SHOES DETECTOR
        
        talk( 'Please be sure to be standing in front of me')
        probss=[]
        head.absolute(human_pos[0],human_pos[1],-0.1)
        rospy.sleep(1.9)
        for i in range (5):
            img=rgbd.get_image()
            cv2.imwrite('feet.png',img)
            print ('got image for feet analysis')
            keys=[ "feet", "shoes",'socks','sandals','sock']
            image = preprocess(Image.fromarray(img)).unsqueeze(0).to(device) #img array from grbd get image
            text = clip.tokenize(keys).to(device)

            with torch.no_grad():
                image_features = model.encode_image(image)
                text_features = model.encode_text(text)
                
                logits_per_image, logits_per_text = model(image, text)
                probs = logits_per_image.softmax(dim=-1).cpu().numpy()

            print("Label probs:", probs,keys[np.argmax(probs)] ,keys, probs[0][1] ) # prints: [[0.9927937  0.00421068 0.00299572]]
        


        head.to_tf('human')
        head.set_joint_values([0,1.3])
        rospy.sleep(0.5)

        if (keys[np.argmax(probs)] in ['sandals','feet','socks', 'sock' ]   ) or ( probs[0][1]  < 0.4  ) :  

            talk ('Thanks for not wearing shoes.... Rule 1 is followed')
            print ('Not wearing shoes')
        else: 
            talk('Rule number 1 Broken... no shoes please... Would you mind taking them off?.... thank you ')
            ##
        return 'succ'


    
#########################################################################################################
class Analyze_area(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : Analyze_area')

        
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
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]

        if len(res.poses.data)==0:
            talk('no Trash  in area next to human....')
            return 'failed'

        else:
            print('object found checking for inflated map')
            
            poses=np.asarray(res.poses.data)
            poses=poses.reshape((int(len(poses)/3) ,3     )      )  
            num_objs=len(poses)
            print (num_objs)
            for i,pose in enumerate(poses):
                #print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                point_name=f'object_{i}'
                tf_man.pub_static_tf(pos=pose, point_name=point_name, ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(point_name=point_name, new_frame='map')
                rospy.sleep(0.3)
                pose,_= tf_man.getTF(point_name)
                print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                    print ('reject point, most likely part of arena, occupied inflated map')
                    tf_man.pub_static_tf(pos=[0,0,0], point_name=point_name, ref='head_rgbd_sensor_rgb_frame')
                    num_objs-=1
                print (f"object found at robot coords.{pose} ")

        if num_objs!=0: 
            head.to_tf('human')
            rospy.sleep(0.5)
            head.set_joint_values([0,1.3])

            rospy.sleep(0.5)
            talk ('Rule number 2 Broken. Garbage detected, would you mind picking it up?')
            self.tries=0
            return 'succ'
        talk('Rule no littering Observed... There is no Trash in area next to human....')
        return 'failed'




        
###########################################################################################################
class Detect_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0
        
    def execute(self,userdata):
        rospy.loginfo('STATE: Detect_drinking')
        print('Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries ==2:
            hv= head.get_joint_values()
            hv[0]= hv[0]+0.6
            head.set_joint_values(hv)
            rospy.sleep(1.0)
        if self.tries ==3:
            hv= head.get_joint_values()
            hv[0]= hv[0]-1.2
            head.set_joint_values(hv)
            rospy.sleep(1.0)
            
        if self.tries == 4:
            talk('Number of tries reached, moving to the next area')
            self.tries=0
            rospy.sleep(0.8)
            return 'failed'
        # Guardo posicion para retornar
        trans, quat=tf_man.getTF('base_link')
        print("TRIES:",self.tries)
        rospy.sleep(0.4)
        if self.tries<2:
            head.set_named_target('neutral')        
            rospy.sleep(1.0)
            hv= head.get_joint_values()
            hv[1]= hv[1]+0.25
            head.set_joint_values(hv)
            rospy.sleep(1.0)
        # ----------------ATENTO EN DONDE ESTARIAN LAS BEBIDAS------------
        place_drinks='drinks_p'
        #-----------------------------------------------------------------
        talk('Detecting drinking')
        rospy.sleep(0.8)
        reqAct.visual=0
        # reqAct.in_ --> 4  para detectar Drinking
        reqAct.in_=4
        resAct=recognize_action(reqAct)

        # 1 -> detecta una persona con drink y pasa a acercarce a la tf correspondiente para ofrecer bebida(?)
        if resAct.i_out==1:
            print(resAct.d_xyz)
            talk('Rule broken, I detect a person without a drink.')
            rospy.sleep(0.8)
            print("Aproaching")
            res = omni_base.move_d_to(1.0,'head_xyz')
            rospy.sleep(1.0)
            head.to_tf('head_xyz')
            rospy.sleep(1.0)

            talk('I detect that you may not have a drink. I will guide you for a drink, please follow me') 
            rospy.sleep(0.8)
            talk(f'Navigating...')
            rospy.sleep(0.8)
            res = omni_base.move_base(known_location=place_drinks,timeout=115)
            #print(res)
            if res :
                rospy.sleep(0.5)
                brazo.set_named_target('neutral')        
                rospy.sleep(1.0)
                talk('Arrived, you can grab a drink here and then come back to the other room, please.')# AGREGAR POINTING A LA TF DONDE ESTAN LAS BEBIDAS (?)
                rospy.sleep(0.8)
                #_ = omni_base.move_base(trans[0],trans[1],tf.transformations.euler_from_quaternion(quat)[2])
                #rospy.sleep(0.8)
                #self.tries=0
                #return 'tries'
            #else:
            #    res = omni_base.move_base(known_location=place_drinks)
            #    rospy.sleep(0.5)
            #    brazo.set_named_target('neutral')        
            #    rospy.sleep(1.0)
            #    talk('Arrived, you can grab a drink here. I will come back to the room.')# AGREGAR POINTING A LA TF DONDE ESTAN LAS BEBIDAS (?)
            #    rospy.sleep(0.8)
            
            _ = omni_base.move_base(trans[0],trans[1],tf.transformations.euler_from_quaternion(quat)[2])
            rospy.sleep(0.8)
            self.tries=0
            return 'tries'
        # 2 -> todos tienen bebida
        elif resAct.i_out==2:
            talk('Ok, someone with a drink.')
            rospy.sleep(0.8)
            #cv2.destroyAllWindows()
            self.tries = 0
            return 'succ'
        # 1 -> La tf salió con NaN, vuelve a calcular y obtener tf de la persona sin drink
        elif resAct.i_out==3:
            talk('No person detected')
            rospy.sleep(0.8)
            return 'tries'
        else:
            talk('Scanning...')
            return 'tries'
    
        
    
###########################################################################################################

# --------------------------------------------------
def init(node_name):
    global reqAct,recognize_action
    print('smach ready')
    reqAct = RecognizeRequest()

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
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           'succ': 'WAIT_PUSH_HAND',   'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    'succ': 'GOTO_NEXT_ROOM',       'tries': 'END'})
        smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',        'succ': 'DETECT_DRINK'    ,   'tries': 'GOTO_NEXT_ROOM', 'forbidden':'GOTO_HUMAN'})
        smach.StateMachine.add("GOTO_HUMAN",         Goto_human(),          transitions={'failed': 'GOTO_HUMAN',        'succ': 'ANALYZE_HUMAN' ,   'tries': 'FIND_HUMAN' , 'forbidden':'LEAD_TO_LIVING_ROOM'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM",Lead_to_living_room(), transitions={'failed': 'LEAD_TO_LIVING_ROOM','succ': 'GOTO_NEXT_ROOM',  'tries': 'END'})
        smach.StateMachine.add("ANALYZE_HUMAN",      Analyze_human(),       transitions={'failed': 'FIND_HUMAN',        'succ': 'ANALYZE_AREA'})
        smach.StateMachine.add("ANALYZE_AREA",      Analyze_area(),         transitions={'failed': 'ANALYZE_AREA' ,     'succ': 'GOTO_NEXT_ROOM' ,    'tries': 'GOTO_NEXT_ROOM'})   #
        smach.StateMachine.add("GOTO_NEXT_ROOM",     Goto_next_room(),      transitions={'failed': 'GOTO_NEXT_ROOM',    'succ': 'FIND_HUMAN'    ,   'tries': 'FIND_HUMAN'})
        #################################################################################################################################################################
        smach.StateMachine.add("DETECT_DRINK",         Detect_drink(),      transitions={'failed': 'GOTO_NEXT_ROOM',    'succ': 'GOTO_HUMAN'  , 'tries': 'FIND_HUMAN'})
        #################################################################################################################################################################
        ##################################################################################################################################################################
        


    outcome = sm.execute()
