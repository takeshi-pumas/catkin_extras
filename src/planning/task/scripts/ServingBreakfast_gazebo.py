#!/usr/bin/env python3

from smach_utils2 import *

#########################################################################################################
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
        
        #READ YAML ROOMS XYS
        
        global arm ,  hand_rgb
        hand_rgb= HAND_RGB()

        arm = moveit_commander.MoveGroupCommander('arm')
        head.set_named_target('neutral')

        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)

        gripper.open()
        rospy.sleep(0.3)

        gripper.close()
        rospy.sleep(0.3)

        
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
        #succ = wait_for_push_hand(100) # NOT GAZEBABLE
        #if succ:
        #    return 'succ'
        #else:
        #    return 'failed'
        return 'succ'


#########################################################################################################
class Goto_pickup(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, pickup')
        res = omni_base.move_base(known_location='pickup', time_out=200)
        print(res)

        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'


#########################################################################################################
class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        #self.target= '024_bowl'
        self.target= '003_cracker_box'
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        talk('Scanning table')
        self.tries += 1
        if self.tries >= 4:self.tries = 0
        if self.tries==1:head.set_joint_values([ 0.0, -0.7])
        if self.tries==2:head.set_joint_values([ 0.2, -0.7])
        if self.tries==3:head.set_joint_values([-0.2, -0.7])
        rospy.sleep(1.3)
        img_msg  = bridge.cv2_to_imgmsg(rgbd.get_image())
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        print (res.poses, res.names)
        if len (res.poses)==0:return 'failed'
        #for pose in res.poses:
        for pose, point_name in zip(res.poses, res.names):
            
            if point_name.data == self.target :
                print ([pose.position.x,pose.position.y,pose.position.z],[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],point_name.data ,'*******************************')
                tf_man.pub_static_tf(pos=[pose.position.x,pose.position.y,pose.position.z], rot =[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w] , point_name='static'+point_name.data, ref='head_rgbd_sensor_depth_frame') #head_rgbd_sensor_rgb_frame')## which object to choose   #TODO
                rospy.sleep(0.3)
                succ=tf_man.change_ref_frame_tf('static'+point_name.data)
                rospy.sleep(0.3)
                print (f'{self.target} found')
                self.target='003_cracker_box'
                return 'succ'
        print( self.target , 'not found')
        return 'failed'

#########################################################################################################
class Pre_pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries', 'pickup_cereal'])
        #self.target = 'bowl'
        self.target = 'cereal'
    def execute(self, userdata):
        rospy.loginfo(f'State : Pre PICKUP  {self.target} ')
        talk(f'Pickup {self.target}')
        
        global target_object, pca_angle
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        clear_octo_client()
        gripper.open()
        
        
        if self.target == 'cereal':
           
            
            #head.to_tf('static003_cracker_box')
            
            head.set_joint_values([0.0 , -0.7])
            
            rospy.sleep(1.0)   #wait for head to get into position  
            request= segmentation_server.request_class() 
            request.height.data= 0.4  #TABLE HEIGHT
            res=segmentation_server.call(request)
            poses=np.asarray(res.poses.data)
            quats=np.asarray(res.quats.data)
            poses=poses.reshape((int(len(poses)/3) ,3     )      )  
            quats=quats.reshape((int(len(quats)/4) ,4     )      )  
            print ( f' {len(poses)}Objects Segmented')
            if len(poses)==0:
                res = omni_base.move_base(known_location='pca_table', time_out=200)
                return 'failed'
            for i,pose in enumerate(poses):
                #print (f'Occupancy map at point object {i}-> pixels ',origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m), img_map[origin_map_img[1]+ round(pose[1]/pix_per_m),origin_map_img[0]+ round(pose[0]/pix_per_m)])
                quat=  quats[i]/np.linalg.norm(quats[i])
                print (np.rad2deg(tf.transformations.euler_from_quaternion(quat)))
                pca_angle=np.sign(np.rad2deg(tf.transformations.euler_from_quaternion(quat))[0])*np.rad2deg(tf.transformations.euler_from_quaternion(quat))[1]
                print( f'################## cereal estimated rotation PCA {pca_angle}')
                print( f'################## cereal estimated Centroid {pose}')
            return 'pickup_cereal'
        
        if self.target== 'bowl':
            target_object='static024_bowl'
            pickup_pose=[0.35,-1.2,0.0,-1.9, 0.0, 0.0]
        
        
        succ= arm.go(pickup_pose)
        rospy.sleep(1.0)
        

        if succ:
            
            succ = False
            
            while not succ:
                #trans,rot = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
                trans,_ = tf_man.getTF(target_frame=target_object, ref_frame='hand_palm_link')
                _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')

                if type(trans) is bool:
                    trans,_ = tf_man.getTF(target_frame='static029_plate', ref_frame='hand_palm_link')
                    _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
                    print('no tf')
                    return 'failed'
                if type(trans) is not bool:
                    eX, eY, eZ = trans
                    eY+=-0.05
                    eT= tf.transformations.euler_from_quaternion(rot)[2] - 0.5*np.pi    #(known loc pickup change to line finder?)   NO! THERE ARE  ROUND TABLES !
                    rospy.loginfo("Distance to goal: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,target_object))
                    if abs(eX) < 0.03:
                        eX = 0
                    if abs(eY) < 0.01:
                        eY = 0
                    if abs(eT   ) < 0.05:
                        eT = 0
                    succ =  eX == 0 and eY == 0 and eT==0
                        # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                    omni_base.tiny_move(velX=0.13*eX, velY=-0.4*eY, velT=-0.3*eT, std_time=0.2, MAX_VEL=0.3) #Pending test
            
            print(tf_man.getTF(ref_frame=target_object,target_frame='base_link'), 'tf obj base######################################')

            img=hand_rgb.get_image()
            img_msg  = bridge.cv2_to_imgmsg(img)
            req      = classify_client.request_class()
            req.in_.image_msgs.append(img_msg)
            res      = classify_client(req)

            obj_name='024_bowl'
            obj_name2='029_plate'
            for i,name in enumerate(res.names):
                print(i,name.data)
                if (name.data== obj_name or name.data== obj_name2 ):break
            print ('BBX',res.pt_min.data[2*i+1],res.pt_max.data[2*i+1],  res.pt_min.data[2*i],res.pt_max.data[2*i])
            #np.save( '~/home/roboworks/Documents/debug_img_hand_yolo.npy',bridge.imgmsg_to_cv2(res.debug_image.image_msgs[0]))
            


            self.target='cereal'
            return 'succ'
            
        return 'failed'

#########################################################################################################
class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()
        self.tries += 1


        if self.tries >= 4:
            self.tries = 0
            return'tries'
        
        clear_octo_client()

        av=arm.get_current_joint_values()
        print (av,'av')
        av[0]+= -0.17
        succ=arm.go(av)
        rospy.sleep(0.5)
        gripper.close()
        print(tf_man.getTF(ref_frame=target_object,target_frame='base_link'), 'tf obj base######################################')
              
        if succ:
            
            tf_man.pub_static_tf(pos=np.zeros(3), rot =[0,0,0,1] , point_name=target_object, ref='erase')
            return 'succ'

            
        return 'failed'

#########################################################################################################
class Pickup_cereal(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('State :  PICKUP ')
        clear_octo_client()
        self.tries += 1


        if self.tries >= 4:
            self.tries = 0
            return'tries'
        

        print ( 'x,y,',np.cos(np.deg2rad(pca_angle)), np.sin(np.deg2rad(pca_angle)) )
        #omni_base.tiny_move(velX=-0.2 *np.cos(np.deg2rad(pca_angle)) , velY= 0.1 * np.sin(np.deg2rad(pca_angle)) ,std_time=4.2, MAX_VEL=0.3) 

      













        clear_octo_client()
        print(pca_angle,max(min((pca_angle+90),-45),45),"####################AnGLE WITHIN LIMITS")
        #print(max(-np.deg2rad(pca_angle),-90),"####################ACNLGE WITHIN LIMITS")
        pickup_pose=[0.077,np.deg2rad(-70),np.deg2rad(-90), np.deg2rad(max(min((pca_angle+90),-45),45)),np.deg2rad(90), 0.0]
        print (pickup_pose)
        succ= arm.go(pickup_pose)
        rospy.sleep(1.0)
        succ=False
        while not succ:
            #trans,rot = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
            trans,_ = tf_man.getTF(target_frame='hand_palm_link', ref_frame='static003_cracker_box')
            _,rot = tf_man.getTF(target_frame='base_link', ref_frame='map')
            print (trans)
            print ('#############')
            if type(trans) is bool:return 'failed'
            if type(trans) is not bool:
                eX, eY, eZ = trans
                eT= tf.transformations.euler_from_quaternion(rot)[2] - 0.5*np.pi    #(known loc pickup change to line finder?)   NO! THERE ARE  ROUND TABLES !
                rospy.loginfo("Distance to goal: {:.2f}, {:.2f}, angle {:.2f}, target obj frame {}".format(eX, eY , eT,'cereal'))
                
                if abs(eX) < 0.05:
                    eX = 0
                if abs(eY) < 0.15:
                    eY = 0
                if abs(eT   ) < 0.1:
                    eT = 0
                succ =  eX == 0 and eY == 0 and eT==0
                    # grasp_base.tiny_move(velY=-0.4*trans[1], std_time=0.2, MAX_VEL=0.3)
                omni_base.tiny_move(velX=-0.1*eY, velY=0.1*eX, velT=-0.3*eT, std_time=0.2, MAX_VEL=0.3) #Pending test

        gripper.close()
        av= arm.get_current_joint_values()
        av[-2]+=-0.2
        arm.go(av)
        rospy.sleep(1.0)
 
        
              
        if succ:
            
            trans,_ = tf_man.getTF(target_frame='static003_cracker_box', ref_frame='hand_palm_link')
            
            print ('#############')
            print ('#############')
            print ('#############')
            print ('#############')
            print (trans, 'hand')
            print ('#############')
            print ('#############')
            print ('#############')
            print ('#############')
            print ('#############')
            
            return 'succ'

            
        return 'failed'

#########################################################################################################
class Post_pickup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('State : Pre PICKUP ')
        clear_octo_client()
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        
        clear_octo_client()

        av=arm.get_current_joint_values()
        print (av,'av')
        av[0]=0.31
        succ=arm.go(av)
        omni_base.tiny_move(velX=-0.2, std_time=1.0, MAX_VEL=0.3) 
        
       
        if succ:
            return 'succ'
            
        return 'failed'

#########################################################################################################
class Goto_table(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        arm.set_named_target('go')
        arm.go()
        


        self.tries += 1
        if self.tries == 3:
            return 'tries'
        if self.tries == 1: talk('Navigating to, pickup')
        res = omni_base.move_base(known_location='table', time_out=200)
        print(res)

        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Place_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('State : PLACE TABLE ')
        clear_octo_client()
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            return'tries'
        place_pose=[0.5, -0.9, 0.0, -1.9, 0.0, 0.0]
        succ=arm.go(place_pose)
        
        if succ:
            omni_base.tiny_move(velX=0.3, std_time=0.7, MAX_VEL=0.3) 
            
            shake=[0.45, -1.1    , 0.0, -1.9, 0.0, 0.0]
            arm.go(shake)                 
            rospy.sleep(0.3)           
            gripper.open()             
            rospy.sleep(0.3)           
            succ=arm.go(place_pose)
            omni_base.tiny_move(velX=-0.2, std_time=0.7, MAX_VEL=0.4) 
            res = omni_base.move_base(known_location='pca_table', time_out=200)
            

            return 'succ'
            
        return 'failed'



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
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    'succ': 'GOTO_PICKUP',       'tries': 'END'})
        smach.StateMachine.add("GOTO_PICKUP",     Goto_pickup(),      transitions={'failed': 'GOTO_PICKUP',    'succ': 'SCAN_TABLE'    ,   'tries': 'INITIAL'})
        smach.StateMachine.add("SCAN_TABLE",        Scan_table(),           transitions={'failed': 'SCAN_TABLE',        'succ': 'PRE_PICKUP'    ,   'tries': 'INITIAL'})  
        smach.StateMachine.add("PRE_PICKUP",        Pre_pickup(),           transitions={'failed': 'PRE_PICKUP',        'succ': 'PICKUP'    ,'pickup_cereal':'PICKUP_CEREAL' ,   'tries': 'END'})        
        smach.StateMachine.add("PICKUP",            Pickup(),           transitions={'failed': 'PICKUP',        'succ': 'POST_PICKUP'    ,   'tries': 'END'})        
        smach.StateMachine.add("PICKUP_CEREAL",     Pickup_cereal(),        transitions={'failed': 'PICKUP_CEREAL',        'succ': 'POST_PICKUP'    ,   'tries': 'END'})        
        smach.StateMachine.add("POST_PICKUP",        Post_pickup(),         transitions={'failed': 'POST_PICKUP',        'succ': 'GOTO_TABLE'    ,   'tries': 'END'})        
        smach.StateMachine.add("GOTO_TABLE",        Goto_table(),           transitions={'failed': 'GOTO_TABLE',        'succ': 'PLACE_TABLE'    ,   'tries': 'END'})        
        smach.StateMachine.add("PLACE_TABLE",     Place_table(),      transitions={'failed': 'GOTO_TABLE',    'succ': 'SCAN_TABLE'    ,   'tries': 'INITIAL'})

        #smach.StateMachine.add("FIND_HUMAN",         Find_human(),          transitions={'failed': 'FIND_HUMAN',        'succ': 'DETECT_DRINK'    ,   'tries': 'GOTO_NEXT_ROOM', 'forbidden':'GOTO_HUMAN'})
        #smach.StateMachine.add("GOTO_HUMAN",         Goto_human(),          transitions={'failed': 'GOTO_HUMAN',        'succ': 'ANALYZE_HUMAN' ,   'tries': 'FIND_HUMAN' , 'forbidden':'LEAD_TO_LIVING_ROOM'})
        #smach.StateMachine.add("LEAD_TO_LIVING_ROOM",Lead_to_living_room(), transitions={'failed': 'LEAD_TO_LIVING_ROOM','succ': 'GOTO_NEXT_ROOM',  'tries': 'END'})
        #smach.StateMachine.add("ANALYZE_HUMAN",      Analyze_human(),       transitions={'failed': 'FIND_HUMAN',        'succ': 'ANALYZE_AREA'})
        #smach.StateMachine.add("ANALYZE_AREA",      Analyze_area(),         transitions={'failed': 'ANALYZE_AREA' ,     'succ': 'GOTO_NEXT_ROOM' ,    'tries': 'GOTO_NEXT_ROOM'})   #
        ##################################################################################################################################################################
        #smach.StateMachine.add("DETECT_DRINK",         Detect_drink(),      transitions={'failed': 'GOTO_NEXT_ROOM',    'succ': 'GOTO_HUMAN'  , 'tries': 'FIND_HUMAN'})
        ##################################################################################################################################################################
        ###################################################################################################################################################################
        


    outcome = sm.execute()
