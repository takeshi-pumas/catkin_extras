#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


################################################################################
#Categories are usually anounced on the competition. Be ready to edit this lists.
################################################################################
def categorize_objs(name):
    kitchen =['bowl','spatula','spoon', 'bowl','plate','f_cups','h_cups']
    tools=['extra_large_clamp','large_clamp','small_clamp','medium_clamp','adjustable_wrench','flat_screwdriver','phillips_screwdriver','wood_block']
    balls= ['softball','tennis_ball','a_mini_soccer_ball', 'racquetball', 'golf_ball', 'baseball'  ]
    fruits= ['apple','banana', 'lemon','pear','plum','orange','strawberry','peach']
    food =['chips_can','mustard_bottle','potted_meat_can','tomato_soup_can','tuna_fish_can','master_chef_can','sugar_box','pudding_box','cracker_box', 'gelatin_box']
    
    if name in kitchen: return 'kitchen'
    elif name in tools: return 'tools'
    elif name in balls: return 'balls'
    elif name in fruits: return 'fruits'
    elif name in food: return 'food'
    return 'other'
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
             
        global arm ,  hand_rgb  ,regions_df , grasping_dict  
        hand_rgb = HAND_RGB()
        ####KNOWLEDGE DATAFRAME 
        rospack = rospkg.RosPack()        
        file_path = rospack.get_path('config_files') 
        objs = pd.read_csv (file_path+'/objects.csv') #EMPTY DATAFRAME
        objs=objs.drop(columns='Unnamed: 0')
        
        grasping_dict = pd.read_csv (file_path+'/GraspingDict.csv')
        print (pd.read_csv (file_path+'/GraspingDict.csv'))
        file_path = rospack.get_path('config_files')
        file_path = rospack.get_path('config_files')+'/regions'         
        
        
        ###############################################
        o=read_yaml('/regions/regions.yaml')#REAL
        ############################################
        #o=read_yaml('/regions/regions_sim.yaml')#SIM
        ###############################################
        regions_df=pd.DataFrame.from_dict(o)
        ############################
        ##TO AVOID SMACH DYING IN CASE NO PLACING AREA IS FOUND, THere is a default that at least allows the test to continue
        #x,y= np.mean(regions['shelves'], axis=0)
        x= 0.5 * (regions_df['shelves']['x_max']-regions_df['shelves']['x_min'])
        y= 0.5 * (regions_df['shelves']['y_max']-regions_df['shelves']['y_min'])
        string_list=regions_df['shelves']['z'].split(',')
        top,mid,low=[list(map(float, s.split(','))) for s in string_list]
        z=mid[0]
        shelf_quat=[regions_df['shelves']['quat_w'],regions_df['shelves']['quat_x'],regions_df['shelves']['quat_y'],regions_df['shelves']['quat_z']]
        tf_man.pub_static_tf(pos=[x,y,z],rot=shelf_quat,point_name='placing_area') ### IF a real placing area is found this tf will be updated
        ################################
        ##########################################
        # Publish all region corner TFs (pickup + shelves)
        ##########################################
        quat = [
            regions_df['shelves']['quat_w'],
            regions_df['shelves']['quat_x'],
            regions_df['shelves']['quat_y'],
            regions_df['shelves']['quat_z']
        ]
        # Region definitions
        region_defs = {
            'pickup': {
                'x_min': regions_df['pickup']['x_min'],
                'x_max': regions_df['pickup']['x_max'],
                'y_min': regions_df['pickup']['y_min'],
                'y_max': regions_df['pickup']['y_max'],
                'z': regions_df['pickup']['z']
            },
            'top': {
                'x_min': regions_df['shelves']['x_min'],
                'x_max': regions_df['shelves']['x_max'],
                'y_min': regions_df['shelves']['y_min'],
                'y_max': regions_df['shelves']['y_max'],
                'z': float(regions_df['shelves']['z'].split(',')[0])  # top
            },
            'mid': {
                'x_min': regions_df['shelves']['x_min'],
                'x_max': regions_df['shelves']['x_max'],
                'y_min': regions_df['shelves']['y_min'],
                'y_max': regions_df['shelves']['y_max'],
                'z': float(regions_df['shelves']['z'].split(',')[1])  # mid
            },
            'low': {
                'x_min': regions_df['shelves']['x_min'],
                'x_max': regions_df['shelves']['x_max'],
                'y_min': regions_df['shelves']['y_min'],
                'y_max': regions_df['shelves']['y_max'],
                'z': float(regions_df['shelves']['z'].split(',')[2])  # low
            }
        }

        for region, vals in region_defs.items():
            corners = [
                (vals['x_min'], vals['y_min']),
                (vals['x_max'], vals['y_min']),
                (vals['x_max'], vals['y_max']),
                (vals['x_min'], vals['y_max'])
            ]
            for i, (x, y) in enumerate(corners):
                tf_man.pub_static_tf(
                    pos=[x, y, vals['z']],
                    rot=quat,
                    point_name=f'{region}_corner_{i}'
                )

        ################################
        arm = moveit_commander.MoveGroupCommander('arm')
        head.set_named_target('neutral')
        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)
        return 'succ'

#########################################################################################################
class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        #############################################################################################
        #return 'succ'## REMOVE  THIS IS ONLY FOR GAZEBO TESTING (no push hand simulated just skip)
        #############################################################################################  
        rospy.loginfo('STATE : Wait for Wait_push_hand')
        print('Waiting for hand to be pushed')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            return 'tries'
        talk('Gently... push my hand to begin')    
        
        succ = wait_for_push_hand(100) # NOT GAZEBABLE
        if succ:
            return 'succ'
        else:
            return 'failed'
###########################################################################################################
class Wait_door_opened(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Wait for door to be opened')
        print('Waiting for door to be opened')

        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')

        # if self.tries == 100:
        #     return 'tries'
        if self.tries == 1 : talk('I am ready for Storing Groceries task.')
        rospy.sleep(0.8)
        talk('I am waiting for the door to be opened')
        isDoor = line_detector.line_found()
        #succ = wait_for_push_hand(100)
        rospy.sleep(1.0)
        if not isDoor:
            self.tries = 0
            return 'succ'
        else:
            return 'failed'

##########################################################
class Goto_pickup(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'],input_keys=['placed_objs'])
        
    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        print('Navigating to, pickup')        
        #talk('Navigating to, pickup')        
        res = omni_base.move_base(known_location='pickup', time_out=40)
        res = omni_base.move_base(known_location='pickup', time_out=40)
        
        print(res)

        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed','tries', 'pour_cereal'], input_keys=['placed_objs'])
        self.tries = 0
        self.scanned=False   
        #self.pickup_plane_z  =0.65 ############REAL    
        
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        
        if self.tries==0:talk('Scanning Table')
        if self.tries==3: return 'tries'
        #self.tries+=1
        global objs
        #if  self.scanned:return'succ'
        rospack = rospkg.RosPack()        
        file_path = rospack.get_path('config_files') 
        objs = pd.read_csv (file_path+'/objects.csv') #EMPTY DATAFRAME
        objs=objs.drop(columns='Unnamed: 0')
        
        
        pickup_plane_z=regions_df['pickup']['z']
        
        #############################AUTOMATIC PLANE HEIGHT DETECTION
        #request= segmentation_server.request_class()   ### Very nice and all but better done offline
        #request.height.data=-1.0   # autodetect planes   # SEGEMENTATION SERVICE
        #res_seg=segmentation_server.call(request)
        #print (f'Planes candidates{res_seg.planes.data}')
        #pickup_plane_z= res_seg.planes.data[0]
        #print (f'Height if max area plane{pickup_plane_z}')
        ########################################################3
        head.set_joint_values([ 0.0, -0.5])
        rospy.sleep(5.0)
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)        
        res      = classify_client(req)
        objects, poses=detect_object_yolo('all',res)   
        if len (objects)!=0 :
            _,quat_base= tf_man.getTF('base_link')  
            #print (regions, self.pickup_plane_z)
            #area_box=regions['pickup']
            #print (area_box)
            
            area_bo_x=regions_df['pickup'][['x_min','x_max']].values
            area_bo_y=regions_df['pickup'][['y_min','y_max']].values
            area_box=np.concatenate((area_bo_x,area_bo_y)).reshape((2,2)).T
            #
            print (area_box)
            def is_inside(x,y,z):return ((area_box[:,1].max() > y) and (area_box[:,1].min() < y)) and ((area_box[:,0].max() > x) and (area_box[0,0].min() < x)) and (pickup_plane_z<z)  
            for i in range(len(res.poses)):
               
                position = [poses[i].position.x ,poses[i].position.y,poses[i].position.z]
               
                ##########################################################

                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                print ('position_map',position_map,'name' ,res.names[i].data[4:])
                if is_inside(position_map.point.x,position_map.point.y,position_map.point.z): 
                    tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=quat_base, ref="map", point_name=res.names[i].data[4:] )
                    new_row = {'x': position_map.point.x, 'y': position_map.point.y, 'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                    objs.loc[len(objs)] = new_row
                    ###########################################################
        else:
            print('Objects list empty')
            talk( 'I could not find more objects')
            self.tries+=1
            return 'failed'
        self.tries=0 
        in_region=[]
        for index, row in objs[['x','y','z']].iterrows():in_region.append(is_inside(row.x, row.y,row.z))
        objs['pickup']=pd.Series(in_region)
        cats=[]
        ######################################################################  Pour Cereal action?
        for name in objs['obj_name']:cats.append(categorize_objs(name))
        objs['category'] = cats    
        objs.dropna(inplace=True)
        objs.to_csv('pickup_objs.csv')
        detected_names = objs['obj_name'].tolist()
        bowl_aliases = ['bowl', 'plate']
        cereal_aliases = ['cereal_box', 'master_chef_can', 'potted_meat_can', 'pudding_box']
        found_bowl = any(name in detected_names for name in bowl_aliases)
        found_cereal = any(name in detected_names for name in cereal_aliases)
        print (f'FOUND CEREAL{found_cereal} FOUND BOWL{found_bowl}: \n\n\n\n')
        if found_bowl and found_cereal and userdata.placed_objs >= 2:
            talk("I found the bowl and the cereal, and I already placed a few groceries. Time to pour some cereal.")
            rospy.loginfo("Both bowl and cereal detected — triggering pour cereal action.")
            return 'pour_cereal'
        #########################################################################
        return 'succ'
#########################################################################################################

class Pickup(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose','mode'], input_keys =['target_object'],outcomes=['succ', 'failed',])
        

    def execute(self, userdata):

        global cat,target_object
        rospy.loginfo('STATE : PICKUP')
        rob_pos,_=tf_man.getTF('base_link')
        pickup_objs=objs[objs['pickup']==True]
        pickup_objs=pickup_objs[pickup_objs['z']>regions_df['pickup']['z']]#PICKUP AREA HEIGHT
        
        print ('pickup_objs',len(pickup_objs['obj_name'].values))
        if  len(pickup_objs['obj_name'].values)==0:
            talk ('No more objects found')
            return 'failed'

        ix=np.argmin(np.linalg.norm(rob_pos-pickup_objs[['x','y','z']]  .values  , axis=1))
        name, cat=pickup_objs[['obj_name','category']].iloc[ix]
        print('closest',name , cat)
        talk ( f'closest object is {name}, of category {cat}, picking it up ')
        
        target_object= name  
        
        line_up_TF(target_object)####################
        print ( 'linning up') #TODO DIctionary ( grasping dict)
        
        string_msg = String()
        #string_msg.data='above'                
        #string_msg.data='frontal'    
        grasp_mode = grasping_dict.loc[grasping_dict['name'] == target_object, 'grasp_mode'].values[0]
        string_msg.data = grasp_mode 
        userdata.mode = string_msg 
        pos, quat = tf_man.getTF(target_frame=target_object, ref_frame='map')
        print(f'target_object {target_object}, mode {string_msg.data}')
        offset_point = grasping_dict[grasping_dict['name'] == target_object][['offset_x', 'offset_y', 'offset_z']].values[0]
        print (f'offset point for {name} is {offset_point }')
        ##################################
        #####################APPLY OFFSET
        object_point = PointStamped()
        object_point.header.frame_id = target_object              #"base_link"
        object_point.point.x = offset_point[0]
        object_point.point.y = offset_point[1]
        object_point.point.z = offset_point[2]+0.01   ### Z OFFSET TO AVOID sweeping table
        transformed_object_point = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
        tf_man.pub_static_tf(pos=[transformed_object_point.point.x,transformed_object_point.point.y,transformed_object_point.point.z],rot=quat,point_name='goal_for_grasp')
        ##################################################################3
        pos, quat = tf_man.getTF(target_frame = 'goal_for_grasp', ref_frame = 'odom')

        pose_goal=np.concatenate((pos,quat))

        print (f'transalted_point-<{pose_goal}')    
        target_pose = Float32MultiArray()
        target_pose.data = pose_goal#pos
        #tf_man.pub_static_tf(pos= translated_point, rot=tf.transformations.quaternion_multiply(quat, [0,0,1,0]), ref="odom", point_name='goal_for_grasp' )   
        rospy.sleep(0.5)
        userdata.target_pose = target_pose
        print (f'transalted_point-<{target_pose.data}')    
        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)       
        clear_octo_client()     
        
        return 'succ'
        
#################################################################################################
class Place(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        global cat,target_object
        rospy.loginfo('STATE : PLACE')

        ##################################################
        
        pos, _ = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'odom')
        target_pose = Float32MultiArray()
        pos[2] += 0.03
        target_pose.data = pos
        userdata.target_pose = target_pose

        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        

        #clear octo client is recommended
        clear_octo_client()               

        return 'succ'        
class Goto_shelf(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries', 'scanned_shelf'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Navigate to known location')
        
        
        
        if "shelves_cats" in globals() and len(shelves_cats)==3:


        
            res=omni_base.move_base(known_location='place_shelf', time_out=35)  
            if res:
                self.tries=0
                return 'scanned_shelf'
            else:
                talk('Navigation Failed, retrying')
                return 'failed'


        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        #omni_base.tiny_move( velX=-0.2,std_time=4.2) 
        #if self.tries == 1: talk('Navigating to, shelf')
        res = omni_base.move_base(known_location='shelf', time_out=60)
        print(res,'res1')
        res = omni_base.move_base(known_location='shelf', time_out=60)

        #res = omni_base.move_base(known_location='shelf', time_out=20)
        print(res,'res2')

        if res:
            self.tries=0
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Goto_place_shelf(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        arm.set_named_target('neutral')    
        arm.set_named_target('go')    
        arm.go()
        #if self.tries == 1: talk('Navigating to, shelf')
        res = omni_base.move_base(known_location='place_shelf', time_out=30)
        res = omni_base.move_base(known_location='place_shelf', time_out=10)
        res = omni_base.move_base(known_location='place_shelf', time_out=10)

        

        print(res)

        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Place_shelf(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'],input_keys=['placed_objs'],
                                                              output_keys=['placed_objs'])
        self.tries = 0
    def execute(self, userdata):
        head.set_joint_values([0,0])
        rospy.loginfo('STATE : Placing in shelf')
        self.tries += 1
        print(f'shelves_cats{shelves_cats}, object picked up cat {cat}')
        ###########################################
        high_shelf_place=[0.632, -1.150, 0.029,  -0.365,0.0, 0.0]        
        mid_shelf_place= [0.132, -1.150, 0.029,  -0.365,0.0, 0.0]
        low_shelf_place= [0.011, -1.666, 0.0, 0.137, 0.0, 0.0]
        #high_shelf_place=[0.669, -1.44, 0.292,  -0.01729,-0.338, 0.0]
        #mid_shelf_place= [0.276, -1.581, 0.15, 0.0621, -0.1234, 0.0]
        #low_shelf_place= [0.05, -2.09, 0.0, 0.455, -0.004, 0.0]
        low_shelf,mid_shelf,top_shelf,=False,False,False
        ###########################################
        string_list=regions_df['shelves']['z'].split(',')
        top,mid,low=[list(map(float, s.split(','))) for s in string_list]

        ############################################
        #placing_places=np.asarray(('placing_area_top_shelf1','placing_area_mid_shelf1','placing_area_low_shelf1'))
        po,_=tf_man.getTF('placing_area')
        if po[2]> top[0]:
            placing_pose=high_shelf_place
            shelf_section='top'
            print('placing at top shelf')
            talk('placing top')

        elif po[2]< mid[0]:
            placing_pose=low_shelf_place
            shelf_section='low'
            print('placing in bottom shelf')
            talk('placing low')
        else:
            placing_pose=mid_shelf_place
            shelf_section='mid'
            print
            talk ('placing at middle shelf')
        ###########################################
        new_row = {
            'x': po[0],
            'y': po[1],
            'z': po[2],
            'obj_name': target_object,
            'shelf_section': shelf_section,  # Instead of separate booleans
            'category': categorize_objs(target_object)  # Categorizing immediately
        }

        objs_shelves.loc[len(objs_shelves)] = new_row  # Append new row correctly

        # Debugging output
        print("############", new_row)
        print(objs_shelves)
        # Optional: Save for debugging (commented out in case you don't want auto-saving)
        # objs_shelves.to_csv('/home/roboworks/Documents/objs.csv', index=False)
        base_grasp_D(tf_name='placing_area',d_x=1.0, d_y=0.0,timeout=10)
        succ=arm.go(placing_pose)
        if succ == False :
            omni_base.tiny_move( velX=-0.1,std_time=1.0) 
            return 'failed'
        hand_grasp_D()  
        #if placing_pose==low_shelf_place:omni_base.tiny_move( velX=0.1,std_time=1.5) 
        rospy.sleep(1.0)
        gripper.open()
        rospy.sleep(1.0)
        head.set_joint_values([0,0])
        omni_base.tiny_move( velX=-0.3,std_time=10.0) 
        arm.set_named_target('go')
        gripper.steady()
        arm.go()
        rospy.sleep(1.5)

        if succ:
            userdata.placed_objs += 1
            self.tries=0
            return'succ'
        return'failed'
        
        
#########################################################################################################
class Check_grasp(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose'],outcomes=['succ', 'failed'])
        self.tries=0

    def execute(self, userdata):
        self.tries+=1
        if self.tries>=4:
            self.tries=0
            
        rospy.loginfo('STATE : Check Grasp')
        hand_img = cv2.cvtColor(hand_rgb.get_image(), cv2.COLOR_BGR2RGB)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(3)
        succ = check_bag_hand_camera(hand_img)
        if succ:
            return 'succ' 
        else: 
            return 'failed'
     
#         succ=brazo.check_grasp()
#         print ( f'brazo check grap result{succ}')
        

#         arm.set_named_target('go')

#         arm.go()
#         head.to_tf(target_object)
#         rospy.sleep(1.0)
#         region_name = "shelf_area"
#         #segmented_img = segment_region(region_name = region_name) 
#         #
#         #img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(segmented_img, cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
#         img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
#         req      = classify_client.request_class()
#         req.in_.image_msgs.append(img_msg)
#         res      = classify_client(req)
#         objects=detect_object_yolo('all',res)   

#         def check_if_grasped(pose_target,test_pt,tolerance=0.05):return np.linalg.norm(pose_target-test_pt)<tolerance
#         ##############################
#         pose_target,_=tf_man.getTF(target_object)
#         #########################

#         if len (objects)!=0 :
#             for i in range(len(res.poses)):                
#                 position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]                
#                 object_point = PointStamped()
#                 object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
#                 object_point.point.x = position[0]
#                 object_point.point.y = position[1]
#                 object_point.point.z = position[2]
#                 position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))                
#                 tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=[0,0,0,1], ref="map", point_name=res.names[i].data[4:] )
#                 new_row = {'x': position_map.point.x, 'y': position_map.point.y, 'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
#                 objs.loc[len(objs)] = new_row                
#                 test_pt=np.asarray((position_map.point.x,position_map.point.y,position_map.point.z))
#                 #print (np.linalg.norm(pose_target-test_pt))
#                 if check_if_grasped(pose_target,test_pt):
#                     print (f'Centroid found in area {test_pt}, obj_name: {res.names[i].data[4:]}')
#                     print ('Grasping May have failed')
#                     talk ('Grasping May have failed')
#                     return 'failed'
        
#         obj_rows = objs[objs['obj_name'] == target_object]

#         if not obj_rows.empty:
#             approx_coords = objs[(objs['x'].round(1) == obj_rows.iloc[0]['x'].round(1)) &
#                                  (objs['y'].round(1) == obj_rows.iloc[0]['y'].round(1)) &
#                                  (objs['z'].round(1) == obj_rows.iloc[0]['z'].round(1))]
#             objs.drop(approx_coords.index, inplace=True)

#         self.tries = 0
#         return'succ'       
#     #########################################################################################################
class Pickup_failed(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : PICKUP FAILED')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        arm.set_named_target('neutral')    
        arm.set_named_target('go')    
        arm.go()
        rospy.sleep(3)
        print(res)
        if res:
            return 'succ'
        
#########################################################################################################   
class Scan_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],output_keys=['missing_shelves'],input_keys=['missing_shelves'])
        self.tries = 0

    def execute(self, userdata):
        global shelves_cats, objs_shelves
        self.tries += 1

        rospack = rospkg.RosPack() 
        file_path = rospack.get_path('config_files')
        file_path = rospack.get_path('config_files')+'/regions'         
        
        
        ###############################################
        o=read_yaml('/regions/regions.yaml')#REAL
        ############################################
        #o=read_yaml('/regions/regions_sim.yaml')#SIM
        ###############################################
        regions_df=pd.DataFrame.from_dict(o)
        ############################
        # Debugging placeholders for category and target object
        #global cat, target_object
        #cat = 'balls'
        #target_object = 'tennis_ball'

        shelf_quat = [regions_df['shelves']['quat_w'], regions_df['shelves']['quat_x'], regions_df['shelves']['quat_y'], regions_df['shelves']['quat_z']]
        print(f'shelf_quat{shelf_quat}')
        string_list = regions_df['shelves']['z'].split(',')
        top, mid, low = [list(map(float, s.split(','))) for s in string_list]
        top_shelf_height = top[0]  # 0.98 #0.8
        mid_shelf_height = mid[0]  # 0.41
        low_shelf_height = low[0]  # 0.01
        # Area box boundaries
        area_bo_x = regions_df['shelves'][['x_min', 'x_max']].values
        area_bo_y = regions_df['shelves'][['y_min', 'y_max']].values
        area_box = np.concatenate((area_bo_x, area_bo_y)).reshape((2, 2)).T
        print(cat, 'category picked up')
        # If shelves_cats has 3 entries, assume the shelf was scanned
        if "shelves_cats" in globals() and len(shelves_cats) == 3:
            talk("I already scanned the shelf.")
            print(shelves_cats, cat, "I already scanned the shelf. Maybe don't scan it every time?")
            corresponding_key = 'low'
            for key, value in shelves_cats.items():
                print(key, value)
                if value == cat:
                    corresponding_key = key  # Iterate over shelves cats and decide
                    print(corresponding_key, key)
            print('corresponding_key', corresponding_key)

        else:
            rospy.loginfo('State: Scanning_shelf')
            clear_octo_client()
            if self.tries==4:  # Too high maybe?
                talk ('Some info is missing, I will move on')
                x, y = np.mean(area_box, axis=0)
                xy_place=[x,y]
                z_place = 0.64
                # Publish TF and move arm
                tf_man.pub_static_tf(pos=[xy_place[0], xy_place[1], z_place + 0.05], point_name='placing_area')
                head.set_joint_values([0, 0])
                rospy.sleep(3)
                arm.set_named_target('go')
                arm.go()
                self.tries=0
                return 'succ'


            talk('Scanning shelf')
            
            if self.tries == 1:
                rospack = rospkg.RosPack()
                if "objs_shelves" not in globals():
                    objs_shelves = pd.DataFrame(columns=['x', 'y', 'z', 'obj_name', 'shelf_section', 'category'])
                
                head.set_joint_values([0.0, 0.0])
                MAX_RETRIES = 10
                av = []

                for i in range(MAX_RETRIES):   ### BELIEVE ITS A GAZEBO ISSUE; but it could happen in real robot
                    av = arm.get_current_joint_values()
                    if av and len(av) >= 2:  # Make sure you can safely assign av[0] and av[1]
                        break
                    rospy.logwarn(f"[WAITING] Could not get valid joint values (try {i+1}/{MAX_RETRIES})")
                    rospy.sleep(0.2)

                if not av or len(av) < 2:
                    rospy.logerr("ERROR: get_current_joint_values() failed — robot state is likely unavailable.")
                    return 'failed'

                # Now safe to use
                av[0] = 0.4
                av[1] = -0.4

                
                arm.go(av)
                head.set_joint_values([-np.pi/2, -0.2])
                rospy.sleep(5)
            elif self.tries == 2:
                print(f"gazing already{userdata.missing_shelves}")
                head.set_joint_values([-np.pi/2, -0.7])
                #arm.set_named_target('go')
                #arm.go()
                #rospy.sleep(5)
            elif self.tries >= 3:
                head.set_joint_values([0.0, 0.0])
                rospy.sleep(2.5)
                arm.set_named_target('go')
                arm.go()
                head.set_joint_values([-np.pi/2, -0.4])
            rospy.sleep(3.0)
            image = cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR)
            img_msg = bridge.cv2_to_imgmsg(image)
            req = classify_client.request_class()
            req.in_.image_msgs.append(img_msg)
            res = classify_client(req)
            debug_image = cv2.cvtColor(bridge.imgmsg_to_cv2(res.debug_image.image_msgs[0]), cv2.COLOR_RGB2BGR)
            objects = detect_object_yolo('all', res)

            def is_inside_shelf(x, y, z):
                if (area_box[1, 1] > y > area_box[0, 1]) and (area_box[1, 0] > x > area_box[0, 0]):
                    if z > top_shelf_height:
                        return "top"
                    elif mid_shelf_height < z < 0.9 * top_shelf_height:
                        return "mid"
                    elif z < 0.9 * mid_shelf_height:
                        return "low"
                return None
            if objects:
                for pose, name in zip(res.poses, res.names):
                    object_point = PointStamped()
                    object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                    object_point.point.x, object_point.point.y, object_point.point.z = pose.position.x, pose.position.y, pose.position.z

                    try:
                        position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                        x, y, z = position_map.point.x, position_map.point.y, position_map.point.z
                        shelf_section = is_inside_shelf(x, y, z)
                        print (f'Name{name} with{x,y,z}coords is deemed in {shelf_section}')

                        if shelf_section:
                            print ('yet Not here= \n\n')
                            tf_man.pub_static_tf(pos=[x, y, z], rot=shelf_quat ,ref="map", point_name=name.data[4:])
                            #tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=quat_base, ref="map", point_name=res.names[i].data[4:] )
                            obj_name = name.data[4:]
                            category = categorize_objs(obj_name)  
                            new_row = {'x': x, 'y': y, 'z': z, 'obj_name': obj_name, 'shelf_section': shelf_section, 'category': category}
                            objs_shelves.loc[len(objs_shelves)] = new_row
                            print(f'Adding new row {new_row}')
                    except Exception as e:
                        print(f"TF transform failed: {e}")
#            if objects:
#                for i in range(len(res.poses)):
#                    position = [res.poses[i].position.x, res.poses[i].position.y, res.poses[i].position.z]
#                    object_point = PointStamped()
#                    object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
#                    object_point.point.x, object_point.point.y, object_point.point.z = position
#                    position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
#                    x, y, z = position_map.point.x, position_map.point.y, position_map.point.z
#                    shelf_section = is_inside_shelf(x, y, z)
#                    if shelf_section:
#                        tf_man.pub_static_tf(pos=[x, y, z], rot=shelf_quat)
#                        new_row = {'x': x, 'y': y, 'z': z, 'obj_name': res.names[i].data[4:], 'shelf_section': shelf_section, 'category': 'other'}
#                        objs_shelves.loc[len(objs_shelves)] = new_row
#                        print(f'adding new row{new_row}')
            else:
                print('no objs')
                return 'failed'

            objs_shelves.dropna(inplace=True)  # Safety measure
            objs_shelves['category'] = objs_shelves['obj_name'].apply(categorize_objs)
            
            shelves_cats = {}
            missing_shelves = []
            print(objs_shelves)
            for shelf in ['top', 'mid', 'low']:
                a = objs_shelves[objs_shelves['shelf_section'] == shelf]['category'].value_counts()
                if 'other' in a:
                    a.drop('other', inplace=True)
                if not a.empty:
                    print(f'{shelf.upper()} shelf category {a.idxmax()}')
                    shelves_cats[shelf] = a.idxmax()
                else:
                    missing_shelves.append(shelf)
            objs_shelves.to_csv("objs_shelves_debug.csv", index=False)
            print(f'shelves_cats, cat{shelves_cats , cat} missing shelves {missing_shelves}')
            print(f'type shelves_cats, cat{shelves_cats.values(),type(cat) } ')
            skip_missing_check = False
            if cat in shelves_cats.values():
                print (print(f'shelves_cats, cat{shelves_cats , cat} EVEN TO missing shelves {missing_shelves}'))
                self.tries=0
                skip_missing_check = True

            if missing_shelves and not skip_missing_check :               
                talk(f'{missing_shelves[0]} shelf missing, scanning again')
                userdata.missing_shelves=missing_shelves
                return 'failed'
        # If already scanned, jump here
        corresponding_key = next((key for key, value in shelves_cats.items() if value == cat), 'low')  # FALLBACK CASE
        print(f'corresponding_key: {corresponding_key}')
        
        # Shelf heights
        shelf_heights = {
            'top': top_shelf_height,
            'mid': mid_shelf_height,
            'low': low_shelf_height
        }
        # Get the placing area by checking the shelf_section
        occupied_pts = objs_shelves[objs_shelves['shelf_section'] == corresponding_key][['x', 'y']].values
        print(f'Objects on {corresponding_key} shelf: {occupied_pts}')
        z_place = shelf_heights[corresponding_key] + 0.1
        xy_place = choose_placing_point(area_box, shelf_quat, occupied_pts)
        tf_man.pub_static_tf(pos=[xy_place[0], xy_place[1], z_place + 0.05], point_name='placing_area')
        head.set_joint_values([0, 0])
        rospy.sleep(3)
        arm.set_named_target('go')
        arm.go()
        objs_shelves.to_csv('shelf_objs.csv')
        self.tries=0
        return 'succ'

#######################################################################################################################################################################        
      

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
    sm.userdata.placed_objs = 0  # initialize placed object countarm=
    # sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),          transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'WAIT_DOOR',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'END'})

        smach.StateMachine.add("WAIT_DOOR",    Wait_door_opened(),      transitions={'failed': 'WAIT_DOOR',    
                                                                                         'succ': 'GOTO_PICKUP'})

        smach.StateMachine.add("GOTO_PICKUP",    Goto_pickup(),         transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'SCAN_TABLE', 
                                                                                         })
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),           transitions={'failed': 'SCAN_TABLE',    
                                                                                         'succ': 'PICKUP',
                                                                                         'tries': 'GOTO_PICKUP',      
                                                                                         'pour_cereal': 'POUR_CEREAL_GOAL'
                                                                                         })        
        smach.StateMachine.add("GOTO_SHELF",    Goto_shelf(),           transitions={'failed': 'GOTO_SHELF',    
                                                                                         'succ': 'SCAN_SHELF',       
                                                                                         'tries': 'GOTO_SHELF',
                                                                                         'scanned_shelf':'SCAN_SHELF'})
        smach.StateMachine.add("GOTO_PLACE_SHELF",    Goto_place_shelf(),       transitions={'failed': 'GOTO_PLACE_SHELF',    
                                                                                         'succ': 'PLACE_SHELF',       
                                                                                         #'succ': 'PLACE',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("PLACE_SHELF",    Place_shelf(),         transitions={'failed': 'PLACE_SHELF',    
                                                                                         'succ': 'GOTO_PICKUP'})
        smach.StateMachine.add("SCAN_SHELF",    Scan_shelf(),           transitions={'failed': 'SCAN_SHELF',    
                                                                                         'succ': 'GOTO_PLACE_SHELF',       
                                                                                         'tries': 'GOTO_SHELF'})        
        smach.StateMachine.add("PICKUP",    Pickup(),                   transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'GRASP_GOAL'})
        smach.StateMachine.add("CHECK_GRASP",    Check_grasp(),         transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'GOTO_SHELF',
                                                                                         })
        smach.StateMachine.add("PLACE",    Place(),                     transitions={'failed': 'PLACE',    
                                                                                         'succ': 'PLACE_GOAL',
                                                                                         'tries': 'GOTO_PLACE_SHELF'})
        
        smach.StateMachine.add("PICKUP_FAILED",    Pickup_failed(),                   transitions={ 'succ': 'GOTO_PICKUP'})
        
        
        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction
                                                               , goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),                     
                                       transitions={'preempted': 'END', 'succeeded': 'CHECK_GRASP', 'aborted': 'SCAN_TABLE'})
        
        smach.StateMachine.add("PLACE_GOAL", SimpleActionState('place_server', GraspAction, goal_slots=['target_pose']),              
                        transitions={'preempted': 'PLACE_SHELF', 'succeeded': 'CHECK_GRASP', 'aborted': 'GOTO_SHELF'})
        
        smach.StateMachine.add("POUR_CEREAL_GOAL", SimpleActionState('pour_cereal', PourCerealAction),
                        transitions={'preempted': 'END', 'succeeded': 'GOTO_PLACE_SHELF', 'aborted': 'END'})


        ###################################################################################################################################################################
        


    outcome = sm.execute()
