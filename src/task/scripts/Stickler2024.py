#!/usr/bin/env python3

from smach_utils2 import *
import torch
import clip
from PIL import Image
################################################(TO UTILS?)
global model , preprocess
device = "cuda" if torch.cuda.is_available() else "cpu"
print(device)
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
        print('[INITIAL]Robot neutral pose')
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
        room_names=['none','bedroom','kitchen','living_room','dining_room','living_room_2','kitchen_2','bedroom_2']
        xys, room_names
        #####
        ####FORBIDEN ROOM 
        forbiden_room='bedroom'
        closest_room='kitchen_drinks'       # a specific TF 

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
        print(f'[WAITPUSHHAND] Try {self.tries} of 4 attempts')
        if self.tries == 4:
            return 'tries'
        talk('Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'


#########################################################################################################
"""class Goto_door(smach.State):  # ADD KNONW LOCATION DOOR
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
"""

#########################################################################################################
class Find_human(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries','forbidden'])
        self.tries = 0
        self.distGaze=[6,6,3,3,3,3]
    def execute(self, userdata):
        

        rospy.loginfo('State : Find_human')
        talk('Scanning the room for humans')
        self.tries += 1

        if self.tries >= 6:
            self.tries = 0
            return'tries'

        if self.tries==1:
            head.set_joint_values([ 0.0, 0.15])
            rospy.sleep(2)
        if self.tries==2:
            head.set_joint_values([1.1, 0.15])
            rospy.sleep(2)
        if self.tries==3:
            head.set_joint_values([1.25, 0.4])
            rospy.sleep(2)
        if self.tries==4:
            head.set_joint_values([-1.1, 0.15])
            rospy.sleep(2)
        if self.tries==5:
            head.set_joint_values([-1.25, 0.4])
            rospy.sleep(2)
            
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]
        humanpose=detect_human_to_tf(self.distGaze[self.tries])  #make sure service is running
        if humanpose== False:
            print ('no human ')
            return 'failed'
            

        else : 
            human_pose,_=tf_man.getTF('human')
            tmp,_=tf_man.getTF('human',ref_frame='base_link')
            print("[FINDHUMAN] DISTANCIA A HUMANO:",np.linalg.norm(tmp))
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
            

            
            print('[FINDHUMAN] room_robot,room_human',room_robot,room_human)
            print ('[FINDHUMAN] px human',px_pose_human)
            print('[FINDHUMAN] room_human',room_human)

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
            
            if room_human == forbiden_room:
                head.to_tf('human')
                talk (f'human found in forbidden room {room_human} ')
                rospy.sleep(0.6)
                talk('I will take him to a valid location')
                self.tries=0
                return 'forbidden'
                
            talk('Human Found')
            if np.linalg.norm(tmp) >= 1 and np.linalg.norm(tmp) < 1.5:
                res = omni_base.move_d_to(np.linalg.norm(tmp),'human')
                rospy.sleep(0.9)
            else:
                res = omni_base.move_d_to(1.3,'human')
                rospy.sleep(0.9)
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

       
        self.tries += 1
        if self.tries==len(room_names):
            self.tries = 0
        
        next_room = room_names[self.tries]
        talk('Navigating to {next_room}')
       

        res = omni_base.move_base(known_location=next_room)
        print("[GOTONEXTROOM] ",res)

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

        print('[GOTOHUMAN] Try', self.tries, 'of 3 attempts')
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

        print('[GOTOHUMAN] getting close to human')
        head.to_tf('human')

        res = omni_base.move_d_to(0.7,'human')

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

        print('[LEADTOLIVINGROOM] Try', self.tries, 'of 3 attempts')
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
class Analyze_shoes(smach.State):           # Talvez una accion por separado?
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Analyze  SHOES')

        head.set_joint_values([0,1.3])
        rospy.sleep(1.0)
        print("[ANALYZESHOES] Checking for shoes... ")    
        
        human_pos,_=tf_man.getTF('human')
        #head.absolute(human_pos[0],human_pos[1],0.1)
        ##### SHOES NO SHOES DETECTOR
        
        talk( 'Please be sure to be standing in front of me')
        probss=[]
        head.absolute(human_pos[0],human_pos[1],-0.1)
        rospy.sleep(1.9)
        for i in range (5):
            img=rgbd.get_image()
            cv2.imwrite(path.expanduser( '~' )+"/Documents/feet.jpg",img)
            print ('[ANALYZESHOES] got image for feet analysis')
            keys=[ "feet", "shoes",'socks','sandals','sock']
            image = preprocess(Image.fromarray(img)).unsqueeze(0).to(device) #img array from grbd get image
            text = clip.tokenize(keys).to(device)

            with torch.no_grad():
                image_features = model.encode_image(image)
                text_features = model.encode_text(text)
                
                logits_per_image, logits_per_text = model(image, text)
                probs = logits_per_image.softmax(dim=-1).cpu().numpy()

            print("[ANALYZESHOES] Label probs:", probs,keys[np.argmax(probs)] ,keys, probs[0][1] ) # prints: [[0.9927937  0.00421068 0.00299572]]
        


        head.to_tf('human')
        head.set_joint_values([0,1.3])
        rospy.sleep(0.5)

        if (keys[np.argmax(probs)] in ['sandals','feet','socks', 'sock' ]   ) or ( probs[0][1]  < 0.4  ) :  

            talk ('Thanks for not wearing shoes.... Rule 1 is followed')
            print('[ANALYZESHOES] Not wearing shoes')
        else: 
            talk('Rule number 1 Broken... no shoes please... Would you mind taking them off?.... thank you ')
            ##
        return 'succ'


#########################################################################################################
class Analyze_trash(smach.State):           # Talvez una accion por separado?
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : TRASH')

        
        self.tries+=1
        #NO MOVEIT
        #hv=head_mvit.get_current_joint_values()
        #hv[0]=1.0
        #head_mvit.go(hv)

        
        
        if self.tries==1: head.set_joint_values([0.5,-0.7])
        elif self.tries==2: head.set_joint_values([-0.5,-0.7])
        elif self.tries==3:
            self.tries=0
            return 'tries'
        rospy.sleep(2.9)
        ##### Segment and analyze
        img=rgbd.get_image()
        cv2.imwrite(path.expanduser( '~' )+"/Documents/rubb.jpg",img)
        print ('[ANALYZETRASH] got image for segmentation')
        res=segmentation_server.call()
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]

        if len(res.poses.data)==0:
            talk('no Trash  in area next to human....')
            return 'failed'

        else:
            print('[ANALYZETRASH] object found checking for inflated map')
            
            poses=np.asarray(res.poses.data)
            poses=poses.reshape((int(len(poses)/3) ,3     )      )  
            num_objs=len(poses)
            print ("[ANALYZETRASH] ",num_objs)
            for i,pose in enumerate(poses):
                #print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                point_name=f'object_{i}'
                tf_man.pub_static_tf(pos=pose, point_name=point_name, ref='head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(point_name=point_name, new_frame='map')
                rospy.sleep(0.3)
                pose,_= tf_man.getTF(point_name)
                print (f'[ANALYZETRASH] Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                    print ('[ANALYZETRASH] reject point, most likely part of arena, occupied inflated map')
                    tf_man.pub_static_tf(pos=[0,0,0], point_name=point_name, ref='head_rgbd_sensor_rgb_frame')
                    num_objs-=1
                print (f'[ANALYZETRASH] object found at robot coords.{pose} ')

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
        self.point_img_pub = rospy.Publisher("/image_topic_name", String, queue_size=1)
        
    def execute(self,userdata):
        rospy.loginfo('STATE: Detect_drinking')
        print('[ANALYZEDRINK] Try', self.tries, 'of 3 attempts')
        self.tries += 1
        """if self.tries ==2:
            hv= head.get_joint_values()
            hv[0]= hv[0]+0.6
            head.set_joint_values(hv)
            rospy.sleep(1.0)
        if self.tries ==3:
            hv= head.get_joint_values()
            hv[0]= hv[0]-1.2
            head.set_joint_values(hv)
            rospy.sleep(1.0)
            """
        if self.tries == 4:
            talk('Number of tries reached, moving to the next area')
            self.tries=0
            rospy.sleep(0.8)
            return 'failed'
        # Guardo posicion para retornar
        trans, quat=tf_man.getTF('base_link')
        print("[ANALYZEDRINK] TRIES:",self.tries)
        rospy.sleep(0.4)
        """if self.tries<2:
            head.set_named_target('neutral')        
            rospy.sleep(1.0)
            hv= head.get_joint_values()
            hv[1]= hv[1]+0.25
            head.set_joint_values(hv)
            rospy.sleep(1.0)"""
        # ----------------ATENTO EN DONDE ESTARIAN LAS BEBIDAS------------
        place_drinks='drinks_p'
        #-----------------------------------------------------------------
        # PENDIENTE DE CAMBIAR LA LOGISTICA DE DETECCION DE BEBIDAS

        # PRIMERO SE ACERCA AL HUMANO
        #human_pose,_=tf_man.getTF('human')
        #print('[ANALYZEDRINK] getting close to human')
        #head.to_tf('human')
        #res = omni_base.move_d_to(0.8,'human')

        # SEGUNDO TOMA IMAGEN Y LA ANALIZA (PENDIENTE SI VA A SER CON UNA NN) PARA VER SI TIENE BEBIDA
        talk('Analysing for drink')
        point_msg = String("DrinkingPose.jpg")
        self.point_img_pub.publish(point_msg)
        rospy.sleep(1.2)
        talk("Three")
        rospy.sleep(1.0)
        talk('Two')
        rospy.sleep(1.0)
        talk('One')
        rospy.sleep(1.0)
        self.point_img_pub.publish(String())
        rospy.sleep(0.1)
        points_msg=rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,timeout=5)
        img,masked_image = removeBackground(points_msg,distance = 2)
        cv2.imwrite(path.expanduser( '~' )+"/Documents/drink.jpg",masked_image)
        print ('[ANALYZEDRINK] got image for drink analysis')
        talk('Ok, analysing...')
        rospy.sleep(0.1)
        # parte de la NN para detectar objeto
        keys=[ "beverage", "food"]
        image = preprocess(Image.fromarray(masked_image)).unsqueeze(0).to(device) #img array from grbd get image
        text = clip.tokenize(keys).to(device)

        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)
            
            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
        talk('Done')
        print("[ANALYZEDRINK] Label probs:", probs,keys[np.argmax(probs)] ,keys, probs[0][1] ) # prints: [[0.9927937  0.00421068 0.00299572]]

        
        rospy.sleep(0.1)

        #
        return 'succ'
        
    
        
###########################################################################################################

# --------------------------------------------------
def init(node_name):
    global reqAct,recognize_action
    print('smach ready')
    #reqAct = RecognizeRequest()

# --------------------------------------------------
# Entry point
if __name__ == '__main__':
    print("[MAIN] Takeshi STATE MACHINE...")
    init("takeshi_smach")
    # State machine, final state "END"
    sm = smach.StateMachine(outcomes=['END'])

    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STICKLER')
    sis.start()

    with sm:
        # State machine STICKLER

        

        smach.StateMachine.add("INITIAL",           
                                Initial(),
                                transitions={'failed': 'INITIAL',
                                               'succ': 'WAIT_PUSH_HAND',   
                                               'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    
                                Wait_push_hand(),       
                                transitions={'failed': 'WAIT_PUSH_HAND',    
                                                'succ': 'GOTO_NEXT_ROOM',       
                                                'tries': 'END'})
        smach.StateMachine.add("FIND_HUMAN",         
                                Find_human(),          
                                transitions={'failed': 'FIND_HUMAN',        
                                                'succ': 'DETECT_DRINK',   
                                                'tries': 'GOTO_NEXT_ROOM', 
                                                'forbidden':'GOTO_HUMAN'})
        smach.StateMachine.add("GOTO_HUMAN",         
                                Goto_human(),          
                                transitions={'failed': 'GOTO_HUMAN',        
                                                'succ': 'ANALYZE_SHOES',   
                                                'tries': 'FIND_HUMAN', 
                                                'forbidden':'LEAD_TO_LIVING_ROOM'})
        smach.StateMachine.add("LEAD_TO_LIVING_ROOM",
                                Lead_to_living_room(), 
                                transitions={'failed': 'LEAD_TO_LIVING_ROOM',
                                                'succ': 'GOTO_NEXT_ROOM',  
                                                'tries': 'END'})
        smach.StateMachine.add("ANALYZE_SHOES",      
                                Analyze_shoes(),       
                                transitions={'failed': 'FIND_HUMAN',        
                                                'succ': 'ANALYZE_TRASH'})
        smach.StateMachine.add("ANALYZE_TRASH",      
                                Analyze_trash(),         
                                transitions={'failed': 'ANALYZE_TRASH' ,     
                                                'succ': 'GOTO_NEXT_ROOM' ,    
                                                'tries': 'GOTO_NEXT_ROOM'})   
        smach.StateMachine.add("GOTO_NEXT_ROOM",     
                                Goto_next_room(),      
                                transitions={'failed': 'GOTO_NEXT_ROOM',    
                                                'succ': 'FIND_HUMAN',   
                                                'tries': 'FIND_HUMAN'})
        smach.StateMachine.add("DETECT_DRINK",         
                                Detect_drink(),      
                                transitions={'failed': 'GOTO_NEXT_ROOM',    
                                                'succ': 'GOTO_HUMAN', 
                                                'tries': 'FIND_HUMAN'})
        ###########################################################################################################
        
        


    outcome = sm.execute()
