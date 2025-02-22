#!/usr/bin/env python3

from smach_utils2 import *
#import torch
#import clip
from PIL import Image
################################################(TO UTILS?)
global model , preprocess
#device = "cuda" if torch.cuda.is_available() else "cpu"
#model, preprocess = clip.load("ViT-B/32", device=device)
classify_drink_client = rospy.ServiceProxy('/classifydrink', Classify)

##### Define state INITIAL #####

#########################################################################################################
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        global forbiden_room,closest_room , xys , room_names ,drinks_room,human_found,roomsDataName
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
        closest_room='allowed_place'
        drinks_room = 'drink_room'
        human_found = False
        head.set_named_target('neutral')
        roomsDataName = 'room_regions_stickler_lab.npy'
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
        self.distGaze=[3.5,2,2,2,2]
        self.gaze = [[ 0.0, 0.15],[1.25, 0.15],[1.4, 0.4],[-1.25, 0.15],[-1.4, 0.4]]
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
            pose=human_pose[:2]
            room_robot,room_human = get_robot_person_coords(pose,fileName=roomsDataName)    # cambiarlo de acuerdo al mapa
            print('[FINDHUMAN] room_robot,room_human',room_robot,room_human)

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
                print("[ANALYSEFORBIDDENROOM] CASO DISTINTO, failed")
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
            print(f'[LEADTOALLOWEDROOM] I will lead you to a valid location, please follow me')
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
            self.confirm = False
            return 'continue'
        else:
            print('[LEADTOALLOWEDROOM] Navigation Failed, retrying')
            return 'failed'

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
 
#########################################################################################################
class Confirm_forbidden(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.distGaze = 3

    def execute(self, userdata):

        rospy.loginfo('STATE :Confirm action in forbidden room')

        print(f'[CONFIRMFORBIDDENROOM] Try {self.tries} of 3 attempts')
        print(f'[CONFIRMFORBIDDENROOM] Scanning if the human found earlier is still in forbidden room')
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
            room_robot,room_human = get_robot_person_coords(human_pose[:2],fileName=roomsDataName)
            print('[CONFIRMFORBIDDENROOM] room_robot,room_human',room_robot,room_human)
            
            if room_human != forbiden_room:
                print("[CONFIRMFORBIDDENROOM] Human found in other room")
                return 'succ'
            
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
class Goto_next_room(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.next_room=1


    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        print("[GOTONEXTROOM] human Found? :",human_found)
        if not(human_found):
            self.tries += 1
        if self.tries==len(room_names):
            self.tries = 1
        
        print("[GOTONEXTROOM] Tries:",self.tries)   # solo para simulacion
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
class Find_human(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.distGaze=[6,6,3,3,3,3]
        self.gaze = [[ 0.0, 0.15],[1.35, 0.15],[1.45, 0.4],[-1.35, 0.15],[-1.45, 0.4]]
    def execute(self, userdata):
        

        rospy.loginfo('State : Find_human')
        print(f'[FINDHUMAN] Intento {self.tries}')
        print('[FINDHUMAN] Scanning the room for humans')
        
        self.tries += 1

        if self.tries >= 6:
            self.tries = 0
            return'tries'

        head.set_joint_values(self.gaze[self.tries-1])
        rospy.sleep(2)
                
        rospy.sleep(1)
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

            room_robot,room_human=get_robot_person_coords(human_pose[:2],load_rooms_areas_stickler(fileName='room_regions_stickler_lab.npy'))
            print('[FINDHUMAN] room_robot,room_human',room_robot,room_human)

            
            if room_robot != room_human: 
                print('[FINDHUMAN] maybe false positive... ignoring... ')
                #talk ('Human is not in the same room as me... Ignoring ')
                #rospy.sleep(1.0)
                return 'failed'
            

            print('[FINDHUMAN] Human Found')
            human_found = True
            return 'succ'    

#########################################################################################################
class Goto_human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to known location')

        print('[GOTOHUMAN] Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
                
        print('[GOTOHUMAN] getting close to human')

        human_pose,_=tf_man.getTF('human')
        tmp,_=tf_man.getTF('human',ref_frame='base_link')
        print(f'[GOTOHUMAN] Distancia al humano {np.linalg.norm(tmp)}')

        if np.linalg.norm(tmp) >= 1 and np.linalg.norm(tmp) < 1.5:
            res = omni_base.move_d_to(np.linalg.norm(tmp),'human')
            rospy.sleep(0.9)
        else:
            res = omni_base.move_d_to(1.3,'human')
            rospy.sleep(0.9)

        if res :
            head.to_tf('human')
            rospy.sleep(0.8)
            self.tries=0
            return 'succ'
            
        else:
            print('[GOTOHUMAN] Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Analyze_trash(smach.State):           # Talvez una accion por separado?
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        

    def execute(self, userdata):

        rospy.loginfo('STATE : TRASH')
        print("[ANALYZETRASH] Analysing for trash")
        
        self.tries+=1
 
        if self.tries==1: head.set_joint_values([0.5,-0.7])
        elif self.tries==2: head.set_joint_values([-0.5,-0.7])
        elif self.tries==3:
            self.tries=0
            return 'tries'
        rospy.sleep(3)
        ##### Segment and analyze
        img=rgbd.get_image()
        print ('[ANALYZETRASH] got image for segmentation')
        save_image(img,name="rubbish")
        request= segmentation_server.request_class() 
        request.height.data=0.00
        res=segmentation_server.call(request)
        img_seg=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])
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

###########################################################################################################
class Detect_drink(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries','continue'])
        self.tries = 0
        self.point_img_pub = rospy.Publisher("/image_topic_name", String, queue_size=1)
        
    def execute(self,userdata):
        rospy.loginfo('STATE: Detect_drinking')
        print('[DETECTDRINK] Try', self.tries, 'of 3 attempts')
        self.tries += 1

        if self.tries == 3:
            print('[ANALYZEDRINK] Number of tries reached, moving to the next area')
            self.tries=0
            rospy.sleep(0.8)
            return 'continue'
        # Guardo posicion para retornar
        print("[ANALYZEDRINK] TRIES:",self.tries)
        
        if self.tries<2:
            head.set_named_target('neutral')        
            rospy.sleep(1.0)
            hv= head.get_joint_values()
            hv[1]= hv[1]+0.25
            head.set_joint_values(hv)
            rospy.sleep(1.0)

        if self.tries ==1:
            print('[ANALYZEDRINK] I will check if you have a drink, please stay in front of me but not too close')
            rospy.sleep(1.5)
            print('[ANALYZEDRINK] If possible, please stay like in any position I am showing you ')
            rospy.sleep(0.3)
            #point_msg = String("DrinkingPose.gif")
            #self.point_img_pub.publish(point_msg)
            rospy.sleep(1.2)
            print("[ANALYZEDRINK] Three")
            rospy.sleep(1.0)
            print('[ANALYZEDRINK] Two')
            rospy.sleep(1.0)
            print('[ANALYZEDRINK] One')
            rospy.sleep(1.0)
            #self.point_img_pub.publish(String())
            rospy.sleep(0.1)
        
        if self.tries ==2:
            print('[ANALYZEDRINK] I may not found a drink, I will try one more time')
            rospy.sleep(3)
            print('[ANALYZEDRINK] If possible, please stay like in any position I showed you earlier')
            rospy.sleep(1.2)
            print("[ANALYZEDRINK] Three")
            rospy.sleep(1.0)
            print('[ANALYZEDRINK] Two')
            rospy.sleep(1.0)
            print('[ANALYZEDRINK] One')
            rospy.sleep(1.0)

        img = rgbd.get_image()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        save_image(img,name="drink")
        img_msg  = bridge.cv2_to_imgmsg(img)
        
        print('[ANALYZEDRINK] Ok, thank you. Analysing...')
        rospy.sleep(0.3)
        
        # parte de la NN para detectar objeto

        """req      = classify_drink_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_drink_client(req)

        print('[ANALYZEDRINK] Done')
        rospy.sleep(0.1)

        print(f'[ANALYZEDRINK] [{res.names}, {res.poses}]')#, res.confidence
        print(f'[ANALYZEDRINK] len res names{len(res.names)}')
        #
        if len(res.names) != 0 :
            print('[ANALYZEDRINK] I believe you do have a drink, thank you. Moving on')
            rospy.sleep(0.3)
            self.retry = False
            self.tries = 0
            return 'succ'
        
        print(f'[ANALYZEDRINK] len res names{len(res.names)} and wrong:{self.wrong}')
        if not(self.retry):
            self.retry = True
            return 'tries'
        else:
            print('[ANALYZEDRINK] My guess is that you do not have a drink in your hand, rule broken')
            rospy.sleep(0.3)
            self.retry = False
            return 'failed'"""
        
        return 'true'           # SOLO PARA SIM Y SI NO SE CUENTA CON YOLO INSTALADO
    
###########################################################################################################
class Lead_to_drinks(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):

        rospy.loginfo('STATE : navigate to drink location')

        print('[LEADTODRINKS] Try', self.tries, 'of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        #_,guest_name = get_waiting_guests()
        head.to_tf('human')
        rospy.sleep(0.5)
        
        print(f'[LEADTODRINKS] Rule not followed, guests should have a drink, please follow me')
        #talk('Rule not followed, guests should have a drink, please follow me')
        rospy.sleep(0.7)
        # talk('Navigating to ,living room')
        res = omni_base.move_base(known_location=drinks_room)
        if res :
            self.tries=0
            print( '[LEADTODRINKS] You can grab a drink here')
            #talk('You can grab a drink here')
            head.set_named_target('go')
            rospy.sleep(0.8)
            head.set_joint_values([1.35, 0.35])     # voltea (casi) hacia donde se espera que le siga el humano
            rospy.sleep(0.8)
            return 'succ'

        else:
            print('[LEADTODRINKS] Navigation Failed, retrying')
            return 'failed'
        
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
        #-------------------
        smach.StateMachine.add("WAIT_PUSH_HAND",    
                                Wait_push_hand(),       
                                transitions={'failed': 'WAIT_PUSH_HAND',    
                                            'succ': 'GOTO_FORBIDDEN',       
                                            'tries': 'END'})
        
        smach.StateMachine.add("GOTO_FORBIDDEN",           
                                Goto_forbidden(),
                                transitions={'failed': 'END',
                                            'succ': 'ANALYSE_FORBIDDEN',   
                                            'tries': 'GOTO_FORBIDDEN'})
        #-------------------
        smach.StateMachine.add("ANALYSE_FORBIDDEN",           
                                Analyse_forbidden(),
                                transitions={'failed': 'END',
                                            'succ': 'LEAD_TO_ALLOWED_ROOM',   
                                            'tries': 'ANALYSE_FORBIDDEN',
                                            'empty': 'GOTO_NEXT_ROOM'})
        #-------------------
        smach.StateMachine.add("CONFIRM_FORBIDDEN",           
                                Confirm_forbidden(),
                                transitions={'failed': 'LEAD_TO_ALLOWED_ROOM',
                                            'succ': 'GOTO_FORBIDDEN',   
                                            'tries': 'CONFIRM_FORBIDDEN'})
        #-------------------
        smach.StateMachine.add("RETURN_TO_FORBIDDEN",           
                                Return_to_forbidden(),
                                transitions={'failed': 'END',
                                            'succ': 'CONFIRM_FORBIDDEN',   
                                            'tries': 'RETURN_TO_FORBIDDEN'})
        #-------------------
        smach.StateMachine.add("LEAD_TO_ALLOWED_ROOM",
                                Lead_to_allowed_room(), 
                                transitions={'failed': 'END',
                                            'succ': 'RETURN_TO_FORBIDDEN',  
                                            'tries': 'LEAD_TO_ALLOWED_ROOM',
                                            'continue':'GOTO_FORBIDDEN'})
        #-------------------
        smach.StateMachine.add("GOTO_NEXT_ROOM",     
                                Goto_next_room(),      
                                transitions={'failed': 'GOTO_NEXT_ROOM',    
                                            'succ': 'FIND_HUMAN',   
                                            'tries': 'GOTO_NEXT_ROOM'})
        #-------------------
        smach.StateMachine.add("FIND_HUMAN",
                                Find_human(),
                                transitions={'failed': 'FIND_HUMAN',        
                                            'succ': 'GOTO_HUMAN',   
                                            'tries': 'GOTO_NEXT_ROOM'})
        #-------------------
        smach.StateMachine.add("GOTO_HUMAN",         
                                Goto_human(),          
                                transitions={'failed': 'GOTO_HUMAN',        
                                            'succ': 'ANALYZE_TRASH',   
                                            'tries': 'FIND_HUMAN'})
        #-------------------
        smach.StateMachine.add("ANALYZE_TRASH",      
                                Analyze_trash(),         
                                transitions={'failed': 'ANALYZE_TRASH' ,     
                                            'succ': 'ANALYZE_SHOES' ,    
                                            'tries': 'ANALYZE_SHOES'}) 
        #-------------------
        smach.StateMachine.add("ANALYZE_SHOES",      
                              Analyze_shoes(),       
                              transitions={'failed': 'DETECT_DRINK',        
                                            'succ': 'DETECT_DRINK'})
        #-------------------
        smach.StateMachine.add("DETECT_DRINK",         
                                Detect_drink(),      
                                transitions={'failed': 'LEAD_TO_DRINK_PLACE',    
                                            'succ': 'GOTO_NEXT_ROOM',
                                            'tries':'DETECT_DRINK', 
                                            'continue': 'GOTO_NEXT_ROOM'})    
        #-------------------
        smach.StateMachine.add("LEAD_TO_DRINK_PLACE",         
                                Lead_to_drinks(),      
                                transitions={'failed': 'LEAD_TO_DRINK_PLACE',    
                                            'succ': 'GOTO_NEXT_ROOM', 
                                            'tries': 'LEAD_TO_DRINK_PLACE'})  
        
        
        ###########################################################################################################
        
        
    outcome = sm.execute()
