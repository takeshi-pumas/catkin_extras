#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray


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
        smach.State.__init__(self,output_keys=['grasping_dict'], outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'        
             
        global arm ,  hand_rgb , objs ,regions_df 
        hand_rgb = HAND_RGB()
        ####KNOWLEDGE DATAFRAME 
        rospack = rospkg.RosPack()        
        file_path = rospack.get_path('config_files') 

        
        objs = pd.read_csv (file_path+'/objects.csv') #EMPTY DATAFRAME
        objs=objs.drop(columns='Unnamed: 0')
        print (objs)
        userdata.grasping_dict = pd.read_csv (file_path+'/GraspingDict.csv')
        print (pd.read_csv (file_path+'/GraspingDict.csv'))
        file_path = rospack.get_path('config_files')+'/regions'         
        o=read_yaml('/regions/regions_sim.yaml')
        regions_df=pd.DataFrame.from_dict(o)
        
        
        ############################
        ##TO AVOID SMACH DYING IN CASE NO PLACING AREA IS FOUND, THere is a default that at least allows the test to continue
        #x,y= np.mean(regions['shelves'], axis=0)
        #z=0.44#self.mid_shelf_height=0.4 shelves heights must also be set on SCAN SHELF  state init section.
        #tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_area') ### IF a real placing area is found this tf will be updated
        ############################
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
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries','pickup'])
        self.tries = 0
        self.First=True

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            self.tries = 0
            return 'tries'
        #talk('Navigating to, pickup')        
        res = omni_base.move_base(known_location='pickup', time_out=40)
        print(res)
##################################################################First TIme Only go        
        if self.tries == 1: 
            if res:
                return 'succ'
            else:
                talk('Navigation Failed, retrying')
                return 'failed'
#########################################################################################################
        if self.tries > 1: 
            #talk('Navigating to, pickup')
            if res:
                return 'pickup'
            else:
                talk('Navigation Failed, retrying')
                return 'failed'
#########################################################################################################
class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0   
        #self.pickup_plane_z  =0.65 ############REAL
        
        
        
    
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        self.tries += 1
        if self.tries >= 3:
            self.tries = 0            
            print ( "I could not find more objects ")
            return 'tries'
        if self.tries==1:
            
            talk('Scanning Table')
        
        global objs 
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
        objects=detect_object_yolo('all',res)   
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
               
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
               
                ##########################################################

                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                print ('position_map',position_map)
                if is_inside(position_map.point.x,position_map.point.y,position_map.point.z): 
                    tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=quat_base, ref="map", point_name=res.names[i].data[4:] )
                    new_row = {'x': position_map.point.x, 'y': position_map.point.y, 'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                    objs.loc[len(objs)] = new_row
                    ###########################################################
                
               
        else:
            print('Objects list empty')
            talk( 'I could not find more objects')
            return 'failed'
        
        #rospack = rospkg.RosPack()
        #file_path = rospack.get_path('config_files')+'/regions'         
        #regions={'shelves':np.load(file_path+'/shelves_region.npy'),'pickup':np.load(file_path+'/pickup_region.npy')}
        #regions={'shelves':np.load(file_path+'/shelves_region_sim.npy'),'pickup':np.load(file_path+'/pickup_region_sim.npy')}
        #area_box=regions[name]
        in_region=[]
        for index, row in objs[['x','y','z']].iterrows():in_region.append(is_inside(row.x, row.y,row.z))
        objs['pickup']=pd.Series(in_region)
        cats=[]
        for name in objs['obj_name']:cats.append(categorize_objs(name))
        objs['category'] = cats    
        objs.dropna(inplace=True)
             
        print (objs)
        pickup_objs=objs[objs['pickup']==True]
        if len (pickup_objs)== 0:
            talk('no more objects found')
            return 'failed'
        self.tries=0  
        return 'succ'
#########################################################################################################

class Pickup(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose','mode'], input_keys =['target_object'],outcomes=['succ', 'failed', 'tries'])
        
        self.tries = 0

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
        
        string_msg= String()
        string_msg.data='above'                
        #string_msg.data='frontal'    
        userdata.mode=string_msg 
        pos, quat = tf_man.getTF(target_frame = target_object, ref_frame = 'map')
        print (f'target_object {target_object}, mode {string_msg.data}')
        ####################
        #inv_quat= tf.transformations.quaternion_inverse(quat)
        print ('pose quat',pos,quat)

            #offset_point=[-0.1,-0.03,0.031]          # Offset relative to object tf
        #offset_point= [-0.1, 0.0  ,0.01]          # Offset relative to object tf FRONTAL
        offset_point= [0.00,-0.0 ,0.08]                  #offset above
          
        #####################APPLY OFFSET
        object_point = PointStamped()
        object_point.header.frame_id = target_object#"base_link"
        object_point.point.x = offset_point[0]
        object_point.point.y = offset_point[1]
        object_point.point.z = offset_point[2]
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
        self.tries+=1  
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
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        

        rospy.loginfo('STATE : Navigate to known location')
        if "shelves_cats" in globals() and len(shelves_cats)==3:
            print (shelves_cats , cat)
            corresponding_key='low'
            
            for key, value in shelves_cats.items():
                if value == cat:corresponding_key = key  #Iterate over shelves cats and decide

            
            
            if corresponding_key=='mid' or  corresponding_key=='low':
                talk ( f"I already scanned the shelf. placing in {corresponding_key} shelf")
                res=omni_base.move_base(known_location='place_shelf', time_out=35)  #OPTIMAL VIEWING POSITION (PREKNOWN)          

                if res:
                    self.tries=0
                    return 'succ'
                else:
                    talk('Navigation Failed, retrying')
                    return 'failed'

        



        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        #omni_base.tiny_move( velX=-0.2,std_time=4.2) 
        #if self.tries == 1: talk('Navigating to, shelf')
        res = omni_base.move_base(known_location='shelf', time_out=20)
        print(res)

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
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):
        rospy.loginfo('STATE : Placing in shelf')
        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        print(f'shelves_cats{shelves_cats}, object picked up cat {cat}')
        ###########################################
        high_shelf_place=[0.669, -1.44, 0.292,  -0.01729,-0.338, 0.0]
        mid_shelf_place= [0.276, -1.581, 0.15, 0.0621, -0.1234, 0.0]
        low_shelf_place= [0.01, -2.09, 0.0, 0.455, -0.004, 0.0]
        low_shelf,mid_shelf,top_shelf,=False,False,False
        ###########################################
        #placing_places=np.asarray(('placing_area_top_shelf1','placing_area_mid_shelf1','placing_area_low_shelf1'))
        po,_=tf_man.getTF('placing_area')
        if po[2]> 0.6:
            placing_pose=high_shelf_place
            top_shelf=True
            print('placing top')

        elif po[2]< 0.3:
            placing_pose=low_shelf_place
            low_shelf=True
            print('placing low')
        else:
            placing_pose=mid_shelf_place
            mid_shelf=True
            print('placing mid')
        
        ###########################################
        new_row = {'x':  po[0], 
           'y':  po[1], 
           'z':  po[2], 
           'obj_name': target_object, 
           'low_shelf': low_shelf,
           'mid_shelf': mid_shelf, 
           'top_shelf': top_shelf, 
           'category': cat}

        objs_shelves.loc[len(objs_shelves)+1]=new_row
        objs_shelves.to_csv('/home/roboworks/Documents/objs.csv') 

        print( "############",new_row,objs_shelves )
        

        base_grasp_D(tf_name='placing_area',d_x=1.25, d_y=0.0,timeout=10)
        succ=arm.go(placing_pose)
        if succ == False :
            omni_base.tiny_move( velX=-0.1,std_time=1.0) 
            return 'failed'
        hand_grasp_D()  
        #if placing_pose==low_shelf_place:omni_base.tiny_move( velX=0.1,std_time=1.5) 

        
        rospy.sleep(1.0)
        gripper.open()
        rospy.sleep(1.0)
        #gripper.steady()
        head.set_joint_values([0,0])
        omni_base.tiny_move( velX=-0.3,std_time=8.0) 
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(1.5)


 

        #base_grasp_D(tf_name='placing_area',d_x=0.6, d_y=0.0,timeout=30)
        #rospy.sleep(1.0)
        #gripper.steady()
        if succ:
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
        succ=brazo.check_grasp()
        print ( f'brazo check grap result{succ}')
        

        arm.set_named_target('go')

        arm.go()
        head.to_tf(target_object)
        rospy.sleep(1.0)
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        objects=detect_object_yolo('all',res)   

        def check_if_grasped(pose_target,test_pt,tolerance=0.05):return np.linalg.norm(pose_target-test_pt)<tolerance
        ##############################
        pose_target,_=tf_man.getTF(target_object)
        #########################

        if len (objects)!=0 :
            for i in range(len(res.poses)):                
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]                
                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))                
                tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=[0,0,0,1], ref="map", point_name=res.names[i].data[4:] )
                new_row = {'x': position_map.point.x, 'y': position_map.point.y, 'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                objs.loc[len(objs)] = new_row                
                test_pt=np.asarray((position_map.point.x,position_map.point.y,position_map.point.z))
                print (np.linalg.norm(pose_target-test_pt))
                if check_if_grasped(pose_target,test_pt):
                    print (f'Centroid found in area {test_pt}, obj_name: {res.names[i].data[4:]}')
                    print ('Grasping May have failed')
                    talk ('Grasping May have failed')
                    print (f'recalling grasp action on coordinates{test_pt} wrt map, converting to odom and action goal slot  ')
                    pos, _ = tf_man.getTF(target_frame = target_object, ref_frame = 'odom')
                    target_pose = Float32MultiArray()
                    pos[2] += 0.03
                    target_pose.data = pos
                    userdata.target_pose = target_pose
                    ###################
                    head.set_named_target('neutral')
                    rospy.sleep(0.5)
                    clear_octo_client()
                    return 'failed'
        
        obj_rows = objs[objs['obj_name'] == target_object]
        approx_coords =  objs[(objs['x'].round(1) == obj_rows.iloc[0]['x'].round(1)) &
                            (objs['y'].round(1) == obj_rows.iloc[0]['y'].round(1)) &
                            (objs['z'].round(1) == obj_rows.iloc[0]['z'].round(1))]
        objs.drop(approx_coords.index,inplace=True) 
        self.tries=0
        return'succ'    
                
                
                
                
                


        
#########################################################################################################   
class Scan_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
       
    def execute(self, userdata):
        global shelves_cats , objs_shelves
        self.tries+=1

        
        #######FOR DEBUGGING REMOVE  s
        #global cat  , target_object     
        #cat= 'balls'
        #target_object='tennis_ball'
        #shelves_cats={}
        #shelves_cats['top']='balls'
        #shelves_cats['mid']='food'
        #shelves_cats['low']='fruits'
        ######
        string_list=regions_df['shelves']['z'].split(',')
        top,mid,low=[list(map(float, s.split(','))) for s in string_list]
        top_shelf_height=top[0]  #0.98 #0.8
        mid_shelf_height=mid[0]  #0.41
        low_shelf_height=low[0]  #0.01    
        ####################################
        area_bo_x=regions_df['shelves'][['x_min','x_max']].values
        area_bo_y=regions_df['shelves'][['y_min','y_max']].values
        area_box=np.concatenate((area_bo_x,area_bo_y)).reshape((2,2)).T
        
        print (cat, 'categoty picked up')
        
        #print('Shelves region',regions['shelves'],'###################################')
        ##################
        
        ####################################
        if "shelves_cats" in globals() and len(shelves_cats)==3:###SHELF ALREADY SCANNED
            talk ( "I already scanned the shelf.")            
            print (shelves_cats , cat,"I already scanned the shelf. Maybe dont scan it every time?")
           
            corresponding_key='low'
            for key, value in shelves_cats.items():
                if value == cat:
                    corresponding_key = key  #Iterate over shelves cats and decide
                    print ('here')
                    print (corresponding_key,key)
                print (key, value)
            print ('corresponding_key',corresponding_key )
            
        else:
    ###########################################################################################            
            rospy.loginfo('State : Scanning_shelf')
            talk('Scanning shelf')
            if self.tries==1:
                rospack = rospkg.RosPack()        
                file_path = rospack.get_path('config_files')
                objs_shelves = pd.read_csv (file_path+'/objects.csv') #EMPTY DATAFRAME
                objs_shelves=objs_shelves.drop(columns='Unnamed: 0')
                ###########################################
                head.set_joint_values([0.0 , 0.0])
                av=arm.get_current_joint_values()
                av[0]=0.67
                av[1]=-0.74
                arm.go(av)
                head.set_joint_values([-np.pi/2 , -0.7])        
                rospy.sleep(2.6)
            if self.tries==2:
                head.set_joint_values([0.0 , 0.0])
                av=arm.get_current_joint_values()
                av[0]=0.0            
                arm.go(av)
                head.set_joint_values([-np.pi/2 , -0.7])        
                rospy.sleep(5)       
            rospy.sleep(3.0)
            ####################################
            image= cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR)
            img_msg  = bridge.cv2_to_imgmsg(image)
            req      = classify_client.request_class()
            req.in_.image_msgs.append(img_msg)
            res      = classify_client(req)
            debug_image=   cv2.cvtColor(bridge.imgmsg_to_cv2(res.debug_image.image_msgs[0]), cv2.COLOR_RGB2BGR)
            objects=detect_object_yolo('all',res)  # list of objects detected objects
            
            
            #def is_inside(x,y,z):    return ((area_box[:,1].max() > y) and (area_box[:,1].min() < y)) and ((area_box[:,0].max() > x) and (area_box[:,0].min() < x)) and (pickup_plane_z<z)  
            def is_inside_top(x,y,z):return ((area_box[1,1] > y) and (area_box[0,1] < y)) and ((area_box[1,0] > x) and (area_box[0,0] < x))    and (top_shelf_height < z )  
            def is_inside_mid(x,y,z):return ((area_box[1,1] > y) and (area_box[0,1] < y)) and ((area_box[1,0] > x) and (area_box[0,0] < x))    and ((0.9*top_shelf_height > z) and (mid_shelf_height < z  )  )  
            def is_inside_low(x,y,z):return ((area_box[1,1] > y) and (area_box[0,1] < y)) and ((area_box[1,0] > x) and (area_box[0,0] < x))    and ((0.9*mid_shelf_height > z  )  )
            #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            if len (objects)!=0 :
                for i in range(len(res.poses)):                
                    position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                    print ('position,name',position,res.names[i].data[4:])
                    ##########################################################
                    object_point = PointStamped()
                    object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                    object_point.point.x = position[0]
                    object_point.point.y = position[1]
                    object_point.point.z = position[2]                
                    position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                    print ('position_map',position_map.point.x,position_map.point.y,position_map.point.z)     
                    ###########################################################################################
                    x,y,z=position_map.point.x,position_map.point.y,position_map.point.z
                    print (is_inside_top(x,y,z),                is_inside_mid(x,y,z),                is_inside_low(x,y,z))
                    new_row = {'x': position_map.point.x, 'y': position_map.point.y, 'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                    print (new_row)
                    objs_shelves.loc[len(objs_shelves)] = new_row
                    

            else:print ('no objs')

            in_region=[]
            in_region_mid=[]
            in_region_low=[]
            cats=[]
            
            for index, row in objs_shelves[['x','y','z']].iterrows():
                in_region.append(is_inside_top(row.x, row.y,row.z))
                in_region_mid.append(is_inside_mid(row.x, row.y,row.z))
                in_region_low.append(is_inside_low(row.x, row.y,row.z))
            
            objs_shelves['top_shelf']=pd.Series(in_region)
            objs_shelves['mid_shelf']=pd.Series(in_region_mid)
            objs_shelves['low_shelf']=pd.Series(in_region_low)
            objs_shelves.dropna(inplace=True)
            
            for name in objs_shelves['obj_name']:cats.append(categorize_objs(name))
            objs_shelves['category'] = cats 
            print (objs_shelves)        
            #objs_shelves.to_csv('/home/roboworks/Documents/objs.csv') 
            
            #########################################################################################
        shelves_cats={}
        a=objs_shelves[objs_shelves['top_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['top_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:
            print(f'TOP  shelf category {a.index[a.argmax()]}')
            shelves_cats['top'] =a.index[a.argmax()]
        #####################################3
        a=objs_shelves[objs_shelves['low_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['low_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:            
            print(f'LOW  shelf category {a.index[a.argmax()]}')
            shelves_cats['low'] =a.index[a.argmax()]
        #####################################3
        a=objs_shelves[objs_shelves['mid_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['mid_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:
            print(f'MID  shelf category {a.index[a.argmax()]}')
            shelves_cats['mid'] =a.index[a.argmax()]           
        #####################################3
               
        
        print (shelves_cats , cat)

        ####################################################################################3
        corresponding_key='low'  # fall back case
        for key, value in shelves_cats.items():
            if value == cat:corresponding_key = key        
        print ('corresponding_key',corresponding_key)
        
        shelf_heights = {
        'top': top_shelf_height,
        'mid': mid_shelf_height,
        'low': low_shelf_height
        }
        print('here',objs_shelves[corresponding_key + '_shelf'])
        objs_shelves.to_csv('/home/roboworks/Documents/objs.csv') 
        z_place = shelf_heights.get(corresponding_key, 0) + 0.1
        
        ################ PLACING AREA ESTIMATION FROM KNOWLEDGE DATA BASE
        y_range = np.arange(area_box[0,1]+0.05, area_box[1,1]-0.015, .15)
        x_range = np.arange(area_box[0,0]+0.05, area_box[1,0]-0.015, .15)
        grid = np.meshgrid(x_range, y_range)
        grid_points = np.vstack([grid[0].ravel(), grid[1].ravel()]).T        
        free_grid = grid_points.tolist()
        # Create a new list without the unwanted elements
        free_grid = [free_pt for free_pt in free_grid if all(
            np.linalg.norm(obj_pt - free_pt) >= 0.153 for obj_pt in objs_shelves[objs_shelves[corresponding_key + '_shelf'] == True][['x', 'y']].values)]
        free_grid=np.asarray((free_grid))
        
        
        if len (free_grid)==0:
            print ( 'no placing area found, using generic safe pose')
            x,y= np.mean(area_box, axis=0)
            z=0.44#self.mid_shelf_height=0.4 shelves heights must also be set on SCAN SHELF  state init section.
            tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_area') ### IF a real placing area is found this tf will be updated
                                                                ##  even if no placing area is found for whatever reason au
        else:            
            print (free_grid)
            summed_ds_to_objs=[]
            for pt in free_grid:summed_ds_to_objs.append(np.linalg.norm(pt-objs_shelves[objs_shelves[corresponding_key+'_shelf']==True][['x','y']].values, axis=1).sum())
            xy_place=free_grid[np.argmax(summed_ds_to_objs)]                        
            print(f'placing_point at {corresponding_key}_shelf  {xy_place} , {z_place}')               
            tf_man.pub_static_tf(pos=[xy_place[0], xy_place[1],z_place+0.05], rot =[0,0,0,1], point_name='placing_area')
        
        head.set_joint_values([0.0 , 0.0])
        rospy.sleep(2.6)
        arm.set_named_target('go')
        arm.go()
        self.tries=0
        return 'succ'
        
      

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
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           
                                                                                         #'succ': 'SCAN_SHELF',   
                                                                                         'succ': 'GOTO_PICKUP',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'END'})

        smach.StateMachine.add("WAIT_DOOR",    Wait_door_opened(),       transitions={'failed': 'WAIT_DOOR',    
                                                                                         'succ': 'GOTO_PICKUP'})

        smach.StateMachine.add("GOTO_PICKUP",    Goto_pickup(),       transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'SCAN_TABLE', 
                                                                                         'tries':'GOTO_PICKUP',                                                                                               
                                                                                         'pickup':'PICKUP'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),       transitions={'failed': 'SCAN_TABLE',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'GOTO_PICKUP'})        
        smach.StateMachine.add("GOTO_SHELF",    Goto_shelf(),       transitions={'failed': 'GOTO_SHELF',    
                                                                                         'succ': 'SCAN_SHELF',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("GOTO_PLACE_SHELF",    Goto_place_shelf(),       transitions={'failed': 'GOTO_PLACE_SHELF',    
                                                                                         'succ': 'PLACE_SHELF',       
                                                                                         #'succ': 'PLACE',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("PLACE_SHELF",    Place_shelf(),       transitions={'failed': 'PLACE_SHELF',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("SCAN_SHELF",    Scan_shelf(),       transitions={'failed': 'SCAN_SHELF',    
                                                                                         'succ': 'GOTO_PLACE_SHELF',       
                                                                                         'tries': 'GOTO_SHELF'})        
        smach.StateMachine.add("PICKUP",    Pickup(),       transitions={'failed': 'PICKUP',    
                                                                                         'succ': 'GRASP_GOAL',       
                                                                                         'tries': 'GOTO_PICKUP'})
        smach.StateMachine.add("CHECK_GRASP",    Check_grasp(),       transitions={'failed': 'GRASP_GOAL',    
                                                                                         'succ': 'GOTO_SHELF',
                                                                                         })
        smach.StateMachine.add("PLACE",    Place(),       transitions={'failed': 'PLACE',    
                                                                                         'succ': 'PLACE_GOAL',
                                                                                         'tries': 'GOTO_PLACE_SHELF'})
        
        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction, goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),                      
        
                               transitions={'preempted': 'END', 'succeeded': 'CHECK_GRASP', 'aborted': 'SCAN_TABLE'})
        
        smach.StateMachine.add("PLACE_GOAL", SimpleActionState('place_server', GraspAction, goal_slots=['target_pose']),              
                        transitions={'preempted': 'PLACE_SHELF', 'succeeded': 'CHECK_GRASP', 'aborted': 'PLACE_SHELF'})
        
        ###################################################################################################################################################################
        


    outcome = sm.execute()
