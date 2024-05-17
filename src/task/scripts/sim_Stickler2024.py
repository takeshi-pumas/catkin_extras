#!/usr/bin/env python3

from smach_utils2 import *
#import torch
#import clip
from PIL import Image
################################################(TO UTILS?)
global model , preprocess
#device = "cuda" if torch.cuda.is_available() else "cpu"
#model, preprocess = clip.load("ViT-B/32", device=device)

##### Define state INITIAL #####

#########################################################################################################
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        global forbiden_room,closest_room , xys , room_names 
        rospy.loginfo('STATE : INITIAL')
        print('[INITIAL] Robot neutral pose')
        head.set_named_target('neutral')        
        rospy.sleep(1.0)
        self.tries += 1
        print(f'[INITIAL] Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'
        
        #READ YAML ROOMS XYS
        df=yaml_to_df()
        xys=[]
        xys.append(df[df['child_id_frame']=='bedroom'][['x','y']].values.ravel())
        xys.append(df[df['child_id_frame']=='living_room'][['x','y']].values.ravel())
        xys.append(df[df['child_id_frame']=='dining_room'][['x','y']].values.ravel())
        xys.append(df[df['child_id_frame']=='kitchen'][['x','y']].values.ravel())
        room_names=['none','kitchen','living_room','dining_room','living_room_2','kitchen_2']   # checar el cuarto prohibido en competencia para adecuar
        #xys, room_names
        #####
        ####FORBIDEN ROOM 
        forbiden_room='bedroom'
        closest_room='kitchen_drinks'

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
        print('[WAITPUSHHAND] Waiting for hand to be pushed')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            return 'tries'
        print('[WAITPUSHHAND] Gently... push my hand to begin')
        succ = wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'

#########################################################################################################
class Goto_forbidden(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to forbidden room')

        print(f'[GOTOFORBIDDENROOM] Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        
        if self.tries == 1: print('[GOTOFORBIDDENROOM] Navigating to, forbidden room')
        res = omni_base.move_base(known_location=forbiden_room)
        print("[GOTOFORBIDDENROOM]",res)

        if res:
            return 'succ'
        else:
            print('[GOTOFORBIDDENROOM] Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Analyse_forbidden(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries','empty'])
        self.tries = 0
        self.distGaze=[3,2,2,2,2]
        self.gaze = [[ 0.0, 0.15],[1.1, 0.15],[1.25, 0.4],[-1.1, 0.15],[-1.25, 0.4]]
    def execute(self, userdata):

        rospy.loginfo('STATE :Analysing forbidden room')

        print(f'[ANALYSEFORBIDDENROOM] Try {self.tries} of 5 attempts')
        print(f'[ANALYSEFORBIDDENROOM] Scanning the room for humans')
        
        self.tries += 1

        if self.tries >= 6:
            self.tries = 0
            return 'empty'

        head.set_joint_values(self.gaze[self.tries-1])
        rospy.sleep(2)

        rospy.sleep(2)    
        humanpose=detect_human_to_tf(self.distGaze[self.tries-1])  #make sure service is running
        
        print("[ANALYSEFORBIDDENROOM] human? :",humanpose)
        #humanpose = False
        if not(humanpose):
            print ('[ANALYSEFORBIDDENROOM] no human ')
            return 'tries'
            

        else : 
            print('[ANALYSEFORBIDDENROOM] Human Found, getting close')

            human_pose,_=tf_man.getTF('human')
            tmp,_=tf_man.getTF('human',ref_frame='base_link')
            
            room_robot,room_human=get_robot_person_coords(human_pose[:2],fileName='room_regions_stickler.npy')
            print("[ANALYSEFORBIDDENROOM] room robot",room_robot," room human",room_human)
            print("[ANALYSEFORBIDDENROOM] DISTANCIA A HUMANO:",np.linalg.norm(tmp))
            if room_robot != room_human :
                print("[ANALYSEFORBIDDENROOM] Probablemente fuera de forbidden, descarto")
                return 'tries'
            elif room_robot == room_human and room_human == forbiden_room:
                print(f'[ANALYSEFORBIDDENROOM] human found in forbidden room  ')
                rospy.sleep(0.6)
                print('[ANALYSEFORBIDDENROOM] I will take him to a valid location')
                if np.linalg.norm(tmp) >= 1 and np.linalg.norm(tmp) < 1.5:
                    res = omni_base.move_d_to(np.linalg.norm(tmp),'human')
                    rospy.sleep(0.9)
                else:
                    res = omni_base.move_d_to(1.3,'human')
                    rospy.sleep(0.9)
                self.tries -= 1
                return 'succ' 
            else:
                print("[ANALYSEFORBIDDENROOM]CASO DISTINTO, failed")
                return 'failed'

#########################################################################################################
class Confirm_forbidden(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.distGaze = 2

    def execute(self, userdata):

        rospy.loginfo('STATE :Confirm action in forbidden room')

        print(f'[CONFIRMFORBIDDENROOM] Try {self.tries} of 3 attempts')
        print(f'[CONFIRMFORBIDDENROOM] Scanning the room for human')
        if self.tries==0:
            head.to_tf('human')
            rospy.sleep(0.5)
        else:
            print("[CONFIRMFORBIDDENROOM] Lifting head")
            hv = head.get_joint_values()
            hv[1] += 0.3
            head.set_joint_values(hv)
            rospy.sleep(2)  

        humanpose=detect_human_to_tf(self.distGaze)  #make sure service is running
        
        print("[CONFIRMFORBIDDENROOM] human? :",humanpose)
        #humanpose = False

        if not(humanpose) and self.tries > 0:
            print ('[CONFIRMFORBIDDENROOM] no human, rule followed ')
            self.tries == 0
            return 'succ'
        
        elif not(humanpose) and self.tries ==0:  
            self.tries+=1
            return 'tries'

        else : 
            print('[CONFIRMFORBIDDENROOM] Human Found, getting close')

            human_pose,_=tf_man.getTF('human')
            tmp,_=tf_man.getTF('human',ref_frame='base_link')
            print("[CONFIRMFORBIDDENROOM] DISTANCIA A HUMANO:",np.linalg.norm(tmp))
            print("[CONFIRMFORBIDDENROOM] Human found (again) in forbidden room")
            if np.linalg.norm(tmp) >= 1 and np.linalg.norm(tmp) < 1.5:
                res = omni_base.move_d_to(np.linalg.norm(tmp),'human')
                rospy.sleep(0.9)
            else:
                res = omni_base.move_d_to(1.3,'human')
                rospy.sleep(0.9)
            self.tries=0

            return 'failed'

#########################################################################################################
class Find_human(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries','forbidden'])
        self.tries = 0
        self.distGaze=[6,6,3,3,3,3]
    def execute(self, userdata):
        

        rospy.loginfo('State : Find_human')
        print(f'[FINDHUMAN] Intento {self.tries}')
        print('[FINDHUMAN] Scanning the room for humans')
        
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
                
        rospy.sleep(1)
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]
        humanpose=detect_human_to_tf(self.distGaze[self.tries])  #make sure service is running
        
        print("[FINDHUMAN] human? :",humanpose)
        #humanpose = False
        if not(humanpose):
            print ('[FINDHUMAN] no human ')

            return 'failed'
            

        else : 
            human_pose,_=tf_man.getTF('human')
            tmp,_=tf_man.getTF('human',ref_frame='base_link')
            print("DISTANCIA A HUMANO:",np.linalg.norm(tmp))
            #pose=human_pose[:2]
            #dists=(pose-np.asarray(xys))
            #human_room=room_names[np.linalg.norm(dists, axis=1).argmin()]
            #print(f'human in {human_room}')
            #robot_pose=get_robot_px()
            #dists=(robot_pose-np.asarray(xys))
            #robot_room=room_names[np.linalg.norm(dists, axis=1).argmin()]
            #print(f'Robot  in {robot_room}')
            
            room_robot,room_human=get_robot_person_coords(human_pose[:2],load_rooms_areas_stickler(fileName='room_regions_stickler_lab.npy'))
            
            ###################

            """living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region = load_rooms_areas_stickler()

            pose=human_pose[:2]
            px_pose_human=np.asarray(([origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]))
            room_human =check_room_px(np.flip(px_pose_human),living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region)
            pose=get_robot_px()
            px_pose_robot=np.asarray((origin_map_img[1]+pose[1],origin_map_img[0]+pose[0]))
        
            room_robot = check_room_px(np.flip(px_pose_robot),living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region)
            """

            
            print('[FINDHUMAN] room_robot,room_human',room_robot,room_human)
            #print ('[FINDHUMAN] px human',px_pose_human)
            #print('[FINDHUMAN] room_human',room_human)

            #########################


            #if img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]!=0:#### Yes axes seem to be "flipped" !=0:
                    #print ('reject point, most likely part of the audience, outside of the arena map')
                    #talk ('Human is close to map edge, maybe a part of the audience')
                    #rospy.sleep(1.0)
                    #return 'failed'
            
            if room_robot != room_human: 
                print('[FINDHUMAN] maybe false positive... ignoring... ')
                #talk ('Human is not in the same room as me... Ignoring ')
                #rospy.sleep(1.0)
                return 'failed'
            
            if room_human==forbiden_room:
                head.to_tf('human')
                print(f'[FINDHUMAN] human found in forbidden room {room_human} ')
                rospy.sleep(0.6)
                print('[FINDHUMAN] I will take him to a valid location')
                self.tries=0
                return 'forbidden'
            

            print('[FINDHUMAN] Human Found, getting close')
            
            if np.linalg.norm(tmp) >= 1 and np.linalg.norm(tmp) < 1.5:
                res = omni_base.move_d_to(np.linalg.norm(tmp),'human')
                rospy.sleep(0.9)
            else:
                res = omni_base.move_d_to(1.3,'human')
                rospy.sleep(0.9)
            
            rospy.sleep(0.5)

            head.to_tf('human')
            rospy.sleep(0.5)
                
            head.set_joint_values([0,1.3])
            rospy.sleep(0.5)
            
            self.tries=0

            return 'succ'    

#########################################################################################################
class Goto_next_room(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.next_room=1
        self.epochs = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        self.epochs += 1
        self.tries += 1
        if self.tries==len(room_names):
            self.tries = 1
        
        print(f'[GOTONEXTROOM] TRIES:{self.tries} y EPOCHS:{self.epochs}')
        if self.epochs == 10:
            return 'tries'

        next_room = room_names[self.tries]
        print(f'[GOTONEXTROOM] Navigating to {next_room}')
       

        res = omni_base.move_base(known_location=next_room)
        print("[GOTONEXTROOM]",res)

        if res :
            return 'succ'

        else:
            print('[GOTONEXTROOM] Navigation Failed, retrying')
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
        
        living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region = load_rooms_areas_stickler()

        pose=human_pose[:2]
        px_pose_human=np.asarray(([origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)]))
        room_human =check_room_px(np.flip(px_pose_human),living_room_px_region,kitchen_px_region,bedroom_px_region,dining_room_px_region)
        if room_human== forbiden_room:
        #if room_names[np.linalg.norm(dists, axis=1).argmin()]== forbiden_room:
            head.to_tf('human')
            print (f'[GOTOHUMAN] Sorry  no one is allowed in the {forbiden_room} ')
            rospy.sleep(0.6)
            
            return 'forbidden'

        print('[GOTOHUMAN] getting close to human')
        head.to_tf('human')

        res = omni_base.move_d_to(0.7,'human')

        if res :
            self.tries=0
            return 'succ'
            
        else:
            print('[GOTOHUMAN] Navigation Failed, retrying')
            return 'failed'

###########################################################################################################
class Lead_to_allowed_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries','continue'])
        self.tries = 0
        self.confirm = False
    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('[LEADTOALLOWEDROOM] Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        #_,guest_name = get_waiting_guests()
        head.to_tf('human')
        rospy.sleep(0.5)
        if not(self.confirm):
            print(f'[LEADTOALLOWEDROOM] I will lead you to the {closest_room}(valid location), please follow me')
            rospy.sleep(0.7)    
        else:
            print(f'[LEADTOALLOWEDROOM] Rule not followed, you can not be here, please follow me')
            rospy.sleep(0.7)
        # talk('Navigating to ,living room')
        res = omni_base.move_base(known_location=closest_room)
        if res and not(self.confirm):
            self.tries=0
            print( '[LEADTOALLOWEDROOM] You can remain here, thank you')
            print("[LEADTOALLOWEDROOM] Returning to confirm ")
            self.confirm = True
            return 'succ'
        elif res and self.confirm:
            print( '[LEADTOALLOWEDROOM] You can remain here, thank you')
            return 'continue'
        else:
            print('[LEADTOALLOWEDROOM] Navigation Failed, retrying')
            return 'failed'

 ###########################################################################################################

###########################################################################################################
class Return_to_forbidden(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to forbidden room')

        print(f'[RETURNTOFORBIDDENROOM] Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: print('[RETURNTOFORBIDDENROOM] Navigating to, forbidden room')
        res = omni_base.move_base(known_location=forbiden_room)
        rospy.sleep(0.8)
        print("[RETURNTOFORBIDDENROOM]",res)

        if res:
            
            return 'succ'
        else:
            print('[RETURNTOFORBIDDENROOM] Navigation Failed, retrying')
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
            
        
        human_pos,_=tf_man.getTF('human')
        #head.absolute(human_pos[0],human_pos[1],0.1)
        ##### SHOES NO SHOES DETECTOR
        
        print( '[ANALYZE SHOES] Please be sure to be standing in front of me')
        probss=[]
        head.absolute(human_pos[0],human_pos[1],-0.1)
        rospy.sleep(1.9)
        for i in range (5):
            img=rgbd.get_image()
            cv2.imwrite(path.expanduser( '~' )+"/Documents/feet.jpg",img)
            print ('[ANALYZE SHOES] got image for feet analysis')
            keys=[ "feet", "shoes",'socks','sandals','sock']
            image = preprocess(Image.fromarray(img)).unsqueeze(0).to(device) #img array from grbd get image
            text = clip.tokenize(keys).to(device)

            with torch.no_grad():
                image_features = model.encode_image(image)
                text_features = model.encode_text(text)
                
                logits_per_image, logits_per_text = model(image, text)
                probs = logits_per_image.softmax(dim=-1).cpu().numpy()

            print("[ANALYZE SHOES] Label probs:", probs,keys[np.argmax(probs)] ,keys, probs[0][1] ) # prints: [[0.9927937  0.00421068 0.00299572]]
        


        head.to_tf('human')
        head.set_joint_values([0,1.3])
        rospy.sleep(0.5)

        if (keys[np.argmax(probs)] in ['sandals','feet','socks', 'sock' ]   ) or ( probs[0][1]  < 0.4  ) :  

            print ('[ANALYZE SHOES] Thanks for not wearing shoes.... Rule 1 is followed')
            print ('[ANALYZE SHOES] Not wearing shoes')
        else: 
            print('[ANALYZE SHOES] Rule number 1 Broken... no shoes please... Would you mind taking them off?.... thank you ')
            ##
        return 'succ'


#########################################################################################################
class Analyze_trash(smach.State):           # Talvez una accion por separado?
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : TRASH')
        print("[ANALYZETRASH] Analysing for trash")
        
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
        #cv2.imwrite(path.expanduser( '~' )+"/Documentos/rubb.jpg",img)
        print ('[ANALYZETRASH] got image for segmentation')
        request= segmentation_server.request_class() 
        request.height.data=0.00
        res=segmentation_server.call(request)
        img_seg=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
        cv2.imwrite(path.expanduser( '~' )+"/Documentos/rubb.jpg",img_seg)
        origin_map_img=[round(img_map.shape[0]*0.5) ,round(img_map.shape[1]*0.5)]

        if len(res.poses.data)==0:
            print('[ANALYZETRASH] no Trash  in area next to human....')
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
                print (f"[ANALYZETRASH] object found at robot coords.{pose} ")

        if num_objs!=0: 
            head.to_tf('human')
            rospy.sleep(0.5)
            head.set_joint_values([0,1.3])

            rospy.sleep(0.5)
            print('[ANALYZETRASH] Rule number 2 Broken. Garbage detected, would you mind picking it up?')
            self.tries=0
            return 'succ'
        print('[ANALYZETRASH] Rule no littering Observed... There is no Trash in area next to human....')
        return 'failed'


###########################################################################################################
class Detect_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0
        self.point_img_pub = rospy.Publisher("/image_topic_name", String, queue_size=1)
        
    def execute(self,userdata):
        rospy.loginfo('STATE: Detect_drinking')
        print('[DETECTDRINK] Try', self.tries, 'of 3 attempts')
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
            print('[DETECTDRINK] Number of tries reached, moving to the next area')
            self.tries=0
            rospy.sleep(0.8)
            return 'failed'
        # Guardo posicion para retornar
        trans, quat=tf_man.getTF('base_link')
        print("[DETECTDRINK] TRIES:",self.tries)
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
        # PENDIENTE DE CAMBIAR LA LOGISTICA DE DETECCION DE BEBIDAS

        # PRIMERO SE ACERCA AL HUMANO
        human_pose,_=tf_man.getTF('human')
        print('[DETECTDRINK] getting close to human')
        head.to_tf('human')
        res = omni_base.move_d_to(0.8,'human')

        # SEGUNDO TOMA IMAGEN Y LA ANALIZA (PENDIENTE SI VA A SER CON UNA NN) PARA VER SI TIENE BEBIDA
        print('[DETECTDRINK] If it is posible, please try to stay like any of this options so I can analyze if you have an object')
        #point_msg = String("DrinkingPose.jpg")
        #self.point_img_pub.publish(point_msg)
        rospy.sleep(1.2)
        print("[DETECTDRINK] Three")
        rospy.sleep(1.0)
        print('[DETECTDRINK] Two')
        rospy.sleep(1.0)
        print('[DETECTDRINK] One')
        rospy.sleep(1.0)
        #self.point_img_pub.publish(String())
        rospy.sleep(0.1)
        img=rgbd.get_image()
        #cv2.imwrite(path.expanduser( '~' )+"/Documents/drink.jpg",img)
        print('[DETECTDRINK] got image for drink analysis')

        return 'succ'
        # parte de la NN para detectar objeto

        #

        """talk('Detecting drinking')
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
                                    return 'tries'"""
    
        
###########################################################################################################

# --------------------------------------------------
def init(node_name):
    global reqAct,recognize_action
    print('[INIT] smach ready')
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
                                               'succ': 'GOTO_FORBIDDEN',   
                                               'tries': 'END'})
        
        smach.StateMachine.add("GOTO_FORBIDDEN",           
                                Goto_forbidden(),
                                transitions={'failed': 'END',
                                               'succ': 'ANALYSE_FORBIDDEN',   
                                               'tries': 'GOTO_FORBIDDEN'})
        
        smach.StateMachine.add("ANALYSE_FORBIDDEN",           
                                Analyse_forbidden(),
                                transitions={'failed': 'END',
                                               'succ': 'LEAD_TO_ALLOWED_ROOM',   
                                               'tries': 'ANALYSE_FORBIDDEN',
                                               'empty': 'GOTO_NEXT_ROOM'})
        
        smach.StateMachine.add("CONFIRM_FORBIDDEN",           
                                Confirm_forbidden(),
                                transitions={'failed': 'LEAD_TO_ALLOWED_ROOM',
                                               'succ': 'GOTO_FORBIDDEN',   
                                               'tries': 'CONFIRM_FORBIDDEN'})
        
        smach.StateMachine.add("RETURN_TO_FORBIDDEN",           
                                Return_to_forbidden(),
                                transitions={'failed': 'END',
                                               'succ': 'CONFIRM_FORBIDDEN',   
                                               'tries': 'RETURN_TO_FORBIDDEN'})
        
        smach.StateMachine.add("LEAD_TO_ALLOWED_ROOM",
                              Lead_to_allowed_room(), 
                              transitions={'failed': 'END',
                                              'succ': 'RETURN_TO_FORBIDDEN',  
                                              'tries': 'LEAD_TO_ALLOWED_ROOM',
                                              'continue':'GOTO_FORBIDDEN'})
        
        smach.StateMachine.add("GOTO_NEXT_ROOM",     
                                Goto_next_room(),      
                                transitions={'failed': 'GOTO_NEXT_ROOM',    
                                                'succ': 'FIND_HUMAN',   
                                                'tries': 'GOTO_FORBIDDEN'})

        smach.StateMachine.add("FIND_HUMAN",
                            Find_human(),
                            transitions={'failed': 'FIND_HUMAN',        
                                              'succ': 'ANALYZE_TRASH',   
                                              'tries': 'GOTO_NEXT_ROOM', 
                                              'forbidden':'GOTO_HUMAN'})

        smach.StateMachine.add("GOTO_HUMAN",         
                              Goto_human(),          
                              transitions={'failed': 'GOTO_HUMAN',        
                                              'succ': 'GOTO_NEXT_ROOM',   
                                              'tries': 'FIND_HUMAN', 
                                              'forbidden':'LEAD_TO_ALLOWED_ROOM'})

        
        
        smach.StateMachine.add("ANALYZE_TRASH",      
                              Analyze_trash(),         
                              transitions={'failed': 'ANALYZE_TRASH' ,     
                                              'succ': 'GOTO_NEXT_ROOM' ,    
                                              'tries': 'GOTO_NEXT_ROOM'}) 
        """smach.StateMachine.add("ANALYZE_SHOES",      
                              Analyze_shoes(),       
                              transitions={'failed': 'FIND_HUMAN',        
                                              'succ': 'ANALYZE_TRASH'})
                
        smach.StateMachine.add("WAIT_PUSH_HAND",    
                                                          Wait_push_hand(),       
                                                          transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                          'succ': 'GOTO_NEXT_ROOM',       
                                                                          'tries': 'END'})
        """  
        
        """smach.StateMachine.add("DETECT_DRINK",         
                                                        Detect_drink(),      
                                                        transitions={'failed': 'GOTO_NEXT_ROOM',    
                                                                        'succ': 'GOTO_HUMAN', 
                                                                        'tries': 'FIND_HUMAN'})"""
        ###########################################################################################################
        
        


    outcome = sm.execute()
