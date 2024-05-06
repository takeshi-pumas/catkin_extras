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
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
    def execute(self, userdata):        
        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries += 1
        print(f'Try {self.tries} of 5 attempts')
        if self.tries == 3:
            return 'tries'        
             
        global arm ,  hand_rgb , objs
        hand_rgb = HAND_RGB()
        ####KNOWLEDGE DATAFRAME 
        rospack = rospkg.RosPack()        
        file_path = rospack.get_path('config_files') 
        objs = pd.read_csv (file_path+'/objects.csv') #EMPTY DATAFRAME
        objs=objs.drop(columns='Unnamed: 0')
        print (objs)
        file_path = rospack.get_path('config_files')+'/regions'         
        regions={'shelves':np.load(file_path+'/shelves_region.npy'),'pickup':np.load(file_path+'/pickup_region.npy')}   ## KNOWN REGIONS
        print (f'Regions for Storing Groceries(Real Robot) {regions}')
        ##TO AVOID SMACH DYING IN CASE NO PLACING AREA IS FOUND, THere is a default that at least allows the test to continue
        x,y= np.mean(regions['shelves'], axis=0)
        z=0.44#self.mid_shelf_height=0.4 shelves heights must also be set on SCAN SHELF  state init section.
        tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_area') ### IF a real placing area is found this tf will be updated
                                                                    ##  even if no placing area is found for whatever reason autonoma can keep going
        ############################
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
        

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        
        #talk('Navigating to, pickup')        
        res = omni_base.move_base(known_location='pickup', time_out=40)
        print(res)    
        talk('Navigating to, pickup ')

        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0     
        self.pickup_plane_z  =0.61
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        clear_octo_client()
        self.tries += 1
        if self.tries >= 3:
            self.tries = 0            
            print ( "I could not find more objects ")
            return 'tries'
        if self.tries==1:
            head.set_joint_values([ 0.0, -0.5])

            talk('Scanning Table')
        if self.tries==2:
            head.set_joint_values([ -0.3, -0.5])

            talk('Scanning Table')
        global objs , pickup_plane_z
        pickup_plane_z=self.pickup_plane_z 
        head.set_joint_values([ 0.0, -0.5])
        rospy.sleep(5.0)                        
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        request= segmentation_server.request_class() 
        req.in_.image_msgs.append(img_msg)
        
        # VERY NICE AND ALL BUT THIS CAN BE DONE OFFLINE 
        #request.height.data=-1.0   # autodetect planes   # SEGEMENTATION SERVICE
        #res_seg=segmentation_server.call(request)
        #print (f'Planes candidates{res_seg.planes.data}')
        #pickup_plane_z= res_seg.planes.data[0]
        #print (f'Height if max area plane{pickup_plane_z}')
        ###############################################
        res      = classify_client(req)
        objects=detect_object_yolo('all',res)   
        if len (objects)!=0 :
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
                tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=[0,0,0,1], ref="map", point_name=res.names[i].data[4:] )
                new_row = {'x': position_map.point.x, 'y': position_map.point.y, 'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                objs.loc[len(objs)] = new_row
                ###########################################################

               
        else:
            print('Objects list empty')
            talk( 'I could not find more objects')
            return 'failed'
        
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('config_files')+'/regions'         
        
        #regions={'shelves':np.load(file_path+'/shelf_sim.npy'),'pickup':np.load(file_path+'/pickup_sim.npy')}  #THE SIMS
        regions={'shelves':np.load(file_path+'/shelves_region.npy'),'pickup':np.load(file_path+'/pickup_region.npy')}

        print (regions)
        def is_inside(x,y,z):return ((area_box[:,1].max()+0.2 > y) and (area_box[:,1].min()-0.2 < y)) and ((area_box[:,0].max() +0.2> x) and (area_box[0,0].min() -0.2 < x)) and (0.9*self.pickup_plane_z<z)  
        for name in regions:
            in_region=[]
            area_box=regions[name]
            for index, row in objs[['x','y','z']].iterrows():in_region.append(is_inside(row.x, row.y,row.z))
            objs[name]=pd.Series(in_region)
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
        smach.State.__init__(self, output_keys=['target_pose'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        global cat,target_object
        rospy.loginfo('STATE : PICKUP')
        rob_pos,_=tf_man.getTF('base_link')
        pickup_objs=objs[objs['pickup']==True]
        pickup_objs=pickup_objs[pickup_objs['z']>pickup_plane_z]#PICKUP AREA HEIGHT        
        print ('pickup_objs',len(pickup_objs['obj_name'].values))
        if  len(pickup_objs['obj_name'].values)==0:
            talk ('No more objects found')
        ix=np.argmin(np.linalg.norm(rob_pos-pickup_objs[['x','y','z']]  .values  , axis=1))  #FIND CLOSEST TABLE OBJECT TO ROBOT
        name, cat=pickup_objs[['obj_name','category']].iloc[ix]                                #state category
        print('closest',name , cat)
        talk ( f'closest object is {name}, of category {cat}, picking it up ')
        target_object= name  
        ##################################################
        

        pos , _ = tf_man.getTF(target_frame = target_object, ref_frame = 'odom')

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
        #res = omni_base.move_base(known_location='place_shelf', time_out=10)
        #res = omni_base.move_base(known_location='place_shelf', time_out=10)

        

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
        if self.tries==2:
            clear_octo_client()

        print(f'shelves_cats{shelves_cats}, object picked up cat {cat}')
        ###########################################
        high_shelf_place=[0.669, -1.44, 0.292,  -0.01729,-0.338, 0.0]
        mid_shelf_place= [0.276, -1.581, 0.15, 0.0621, -0.1234, 0.0]
        low_shelf_place= [0.01, -2.09, 0.0, 0.455, -0.004, 0.0]
        ###########################################
        #placing_places=np.asarray(('placing_area_top_shelf1','placing_area_mid_shelf1','placing_area_low_shelf1'))
        po,_=tf_man.getTF('placing_area' )
        if po[2]> 0.6:
            placing_pose=high_shelf_place
            placing_shelf='high'
        elif po[2]< 0.2:
            placing_shelf='bottom'
            placing_pose=low_shelf_place
        else:
            placing_pose=mid_shelf_place
            placing_shelf='middle'
        
        ###########################################
        talk ( f'placing object on  {placing_shelf} shelf') 
        
       

        base_grasp_D(tf_name='placing_area',d_x=0.8+0.05*self.tries , d_y=0.0,timeout=10)
        succ=arm.go(placing_pose)
        if succ == False :
            omni_base.tiny_move( velX=-0.1,std_time=1.0)
            succ=arm.go(placing_pose) 
            if not succ:
                return 'failed'
        hand_grasp_D()  
        if placing_pose==low_shelf_place:omni_base.tiny_move( velX=0.21,std_time=2.0) 

        
        rospy.sleep(1.0)
        gripper.open()
        rospy.sleep(1.0)
        #gripper.steady()
        head.set_joint_values([0,0])
        omni_base.tiny_move( velX=-0.3,std_time=8.0) 
        gripper.steady()
        arm.set_named_target('go')
        arm.go()
        #gripper.steady()
        if succ:
            self.tries=0
            return'succ'
        return'failed'
        
        
#########################################################################################################
class Check_grasp(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose'],outcomes=['succ', 'failed', 'tries'])
        self.tries=0

    def execute(self, userdata):
        self.tries+=1
        if self.tries>=4:
            self.tries=0
            return 'tries'
        rospy.loginfo('STATE : Check Grasp')
        if brazo.check_grasp(): print ( f'brazo check grap result True')        
        else: 
            talk ('grasping might have failed')
            return 'failed'

        arm.set_named_target('go')
        arm.go()
        head.to_tf(target_object)
        rospy.sleep(1.0)
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        objects=detect_object_yolo('all',res)   
        def check_if_grasped(pose_target,test_pt,tolerance=0.1):return np.linalg.norm(pose_target-test_pt)<tolerance
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
        #objs.drop(objs[objs['obj_name'] == target_object].index, inplace=True)
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
        self.top_shelf_height=0.87
        self.mid_shelf_height=0.41
        self.low_shelf_height=0.01    
       
    def execute(self, userdata):
        global shelves_cats , objs_shelves
        self.tries+=1
        



        #######FOR DEBUGGING REMOVE  
        #global cat      
        #cat= 'food'
        #shelves_cats={}
        #shelves_cats['top']='balls'
        #shelves_cats['mid']='food'
        #shelves_cats['low']='fruits'
        ######
        if "shelves_cats" in globals() and len(shelves_cats)==3:
            talk ( "I already scanned the shelf. Maybe dont scan it every time?")
            print (shelves_cats , cat)
            corresponding_key='low'
            for key, value in shelves_cats.items():
                if value == cat:corresponding_key = key  #Iterate over shelves cats and decide
            if corresponding_key=='top':

                talk(f'Category {cat} found in top shelf... Placing')
                print ('GOTO PLACE SHELF TOP')
                head.set_joint_values([0.0 , 0.0])
                rospy.sleep(2.5)
                clear_octo_client()
                av=arm.get_current_joint_values()
                av[0]=0.67
                av[1]=-0.74
                arm.go(av)
                #head.set_joint_values([-np.pi/2 , -0.7])  
                head.to_tf('top_shelf')
                rospy.sleep(2.6)
                rospy.sleep(2.5) 
                omni_base.tiny_move( velY=-0.3,std_time=20.0)              
                
                if find_placing_area(self.top_shelf_height) != True:
                    omni_base.tiny_move( velY=-0.2,std_time=6.66)
                    head.to_tf('top_shelf')
                    find_placing_area(self.top_shelf_height )

                
                clear_octo_client()
                head.set_joint_values([0.0 , 0.0])
                rospy.sleep(2.6)
                rospy.sleep(2.6)
                arm.set_named_target('go')
                arm.go()
                rospy.sleep(2.5)         
                return 'succ'

            elif corresponding_key=='low':

                talk(f'Category {cat} found in low shelf... Placing')
                print ('GOTO PLACE SHELF LOW')
                head.set_joint_values([0.0 , 0.0])
                arm.set_named_target('go')
                arm.go()                
                #omni_base.move_base(known_location='place_shelf', time_out=10)
                #omni_base.move_base(known_location='place_shelf', time_out=10)
                
                
                
                head.to_tf('low_shelf')
                rospy.sleep(2.5)
                if find_placing_area(self.low_shelf_height) != True:
                    omni_base.tiny_move( velX=0.2,std_time=8.5)
                    head.to_tf('low_shelf')
                    find_placing_area(self.low_shelf_height) 
                clear_octo_client()
                rospy.sleep(2.6)
                arm.set_named_target('go')
                arm.go()                
                 
                return 'succ'

            else:
                talk(f'Category {cat} found in mid shelf... Placing')

                head.set_joint_values([0.0 , 0.0])# 
                arm.set_named_target('go')
                arm.go()
                rospy.sleep(1.0)
                
                rospy.sleep(2.5)
                rospy.sleep(2.5)            
                omni_base.tiny_move( velX=0.2,std_time=5.5)
                head.to_tf('mid_shelf')     
                rospy.sleep(2.6)
                if find_placing_area(self.mid_shelf_height) != True:
                    omni_base.tiny_move( velX=0.4,std_time=4)
                    head.to_tf('mid_shelf')
                    find_placing_area(self.mid_shelf_height) 


                    
                head.set_joint_values([0.0 , 0.0])                
                rospy.sleep(2.6)
                arm.set_named_target('go')
                arm.go()
                omni_base.tiny_move( velX=-0.3,std_time=4.0)
                return 'succ'
                

        rospy.loginfo('State : Scanning_shelf')
        talk('Scanning shelf')
        clear_octo_client()
        ##################
        rospack = rospkg.RosPack()                    
        file_path = rospack.get_path('config_files')+'/regions'         
        #regions={'shelves':np.load(file_path+'/shelf_sim.npy'),'pickup':np.load(file_path+'/pickup_sim.npy')}
        regions={'shelves':np.load(file_path+'/shelves_region.npy'),'pickup':np.load(file_path+'/pickup_region.npy')}
        print('Shelves region',regions['shelves'],'###################################')
        ####################################
        x,y= np.mean(regions['shelves'], axis=0)
        tf_man.pub_static_tf(pos=[x,y,self.top_shelf_height],point_name='top_shelf')
        tf_man.pub_static_tf(pos=[x,y,self.mid_shelf_height],point_name='mid_shelf')
        tf_man.pub_static_tf(pos=[x,y,self.low_shelf_height],point_name='low_shelf')
        
    
        file_path = rospack.get_path('config_files') 
        objs_shelves = pd.read_csv (file_path+'/objects.csv') #EMPTY DATAFRAME
        objs_shelves=objs_shelves.drop(columns='Unnamed: 0')
        ###########################################
        head.set_joint_values([0.0 , 0.0])
        rospy.sleep(2.5)  ## wait for head beofre moving hand
        av=arm.get_current_joint_values()
        av[0]=0.67
        av[1]=-0.74
        arm.go(av)
        head.set_joint_values([-np.pi/2 , -0.7])        
        rospy.sleep(2.6)
        rospy.sleep(3.0)
        image= cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR)
        img_msg  = bridge.cv2_to_imgmsg(image)
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        debug_image=   cv2.cvtColor(bridge.imgmsg_to_cv2(res.debug_image.image_msgs[0]), cv2.COLOR_RGB2BGR)
        objects=detect_object_yolo('all',res)  # list of objects detected objects
        ####################################
        area_box=regions['shelves']
        print (area_box)
        def is_inside_top(x,y,z):return ((area_box[:,1].max()+0.15 > y) and (area_box[:,1].min()-0.15 < y)) and ((area_box[:,0].max()+0.15 > x) and (area_box[0,0].min()-0.15 < x))    and (0.9*self.top_shelf_height < z )  
        def is_inside_mid(x,y,z):return ((area_box[:,1].max()+0.15 > y) and (area_box[:,1].min()-0.15 < y)) and ((area_box[:,0].max()+0.15 > x) and (area_box[0,0].min()-0.15 < x))    and ((0.9*self.top_shelf_height > z) and (self.mid_shelf_height < z  )  )  
        def is_inside_low(x,y,z):return ((area_box[:,1].max()+0.15 > y) and (area_box[:,1].min()-0.15 < y)) and ((area_box[:,0].max()+0.15 > x) and (area_box[0,0].min()-0.15 < x))    and ((0.9*self.mid_shelf_height > z  )  )
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



        shelves_cats={}
        a=objs_shelves[objs_shelves['top_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['top_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:
            print(f'TOP  shelf category {a.index[a.argmax()]}')
            shelves_cats['top'] =a.index[a.argmax()]
            


        a=objs_shelves[objs_shelves['low_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['low_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:            
            print(f'LOW  shelf category {a.index[a.argmax()]}')
            shelves_cats['low'] =a.index[a.argmax()]
            
        a=objs_shelves[objs_shelves['mid_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['mid_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:
            print(f'MID  shelf category {a.index[a.argmax()]}')
            shelves_cats['mid'] =a.index[a.argmax()]
            
        
        print (shelves_cats , cat)
        corresponding_key='low'  # fall back case
        for key, value in shelves_cats.items():
            if value == cat:corresponding_key = key        
        print ('corresponding_key',corresponding_key)
        
        if corresponding_key=='top':
            talk(f'Category {cat} found in top shelf... Placing')
            print ('GOTO PLACE SHELF TOP')
            omni_base.tiny_move( velY=-0.3,std_time=15.0)
            #omni_base.move_base(known_location='high_place_area', time_out=10)
            #omni_base.move_base(known_location='high_place_area', time_out=10)
            #omni_base.move_base(known_location='shelf', time_out=10)
            #omni_base.move_base(known_location='shelf', time_out=10)
            #head.set_joint_values([0.0 , 0.0])
            #av=arm.get_current_joint_values()
            #av[0]=0.67
            #av[1]=-0.74
            #arm.go(av)
            #head.set_joint_values([-np.pi/2 , -0.7])        
            #rospy.sleep(5.0)     
            if find_placing_area(self.top_shelf_height):
                head.set_joint_values([0.0 , 0.0])
                rospy.sleep(2.6)
                return 'succ'
            else :
                omni_base.tiny_move( velY=-0.3,std_time=10.0)
                find_placing_area(self.top_shelf_height)
                head.set_joint_values([0.0 , 0.0])
                rospy.sleep(5.0)
                return 'succ'
        
        elif corresponding_key=='mid':
            talk(f'Category {cat} found in  middle shelf... Placing')
            head.set_joint_values([0.0 , 0.0])
            arm.set_named_target('go')
            arm.go()
            rospy.sleep(1.0)
            #omni_base.move_base(known_location='bottom_place_area', time_out=10)
            #omni_base.move_base(known_location='bottom_place_area', time_out=10)
            omni_base.move_base(known_location='place_shelf', time_out=10)
            omni_base.move_base(known_location='place_shelf', time_out=10)
            rospy.sleep(2.5)
            rospy.sleep(2.5)            
            head.set_joint_values([0.0 , -0.7])        
            rospy.sleep(2.6)
            omni_base.tiny_move( velX=0.2,std_time=5.0) 
            if find_placing_area(self.mid_shelf_height):
                head.set_joint_values([0.0 , 0.0])
                rospy.sleep(2.6)
                arm.set_named_target('go')
                arm.go()
                return 'succ'
            omni_base.tiny_move( velX=0.2,std_time=5.0) 
            if find_placing_area(self.mid_shelf_height):
                head.set_joint_values([0.0 , 0.0])
                rospy.sleep(2.6)
                arm.set_named_target('go')
                arm.go()
                return 'succ'                
                
            omni_base.tiny_move( velX=0.2,std_time=5.0) 
            if find_placing_area(self.mid_shelf_height):
                head.set_joint_values([0.0 , 0.0])
                rospy.sleep(2.6)
                arm.set_named_target('go')
                arm.go()
                return 'succ'                
            return 'failed'
            

            return 'succ'

        else:
            talk(f'Category {cat} found in low shelf... Placing')
            print ('GOTO PLACE SHELF LOW')
            head.set_joint_values([0.0 , 0.0])
            arm.set_named_target('go')
            arm.go()
            rospy.sleep(1.0)
            #omni_base.move_base(known_location='bottom_place_area', time_out=10)
            #omni_base.move_base(known_location='bottom_place_area', time_out=10)
            omni_base.move_base(known_location='place_shelf', time_out=10)
            rospy.sleep(2.5)
            rospy.sleep(2.5)
            omni_base.tiny_move( velX=0.1,std_time=5.0) 
            head.to_tf('low_shelf')
            rospy.sleep(2.5)
            if find_placing_area(self.low_shelf_height):
                return 'succ'
            omni_base.tiny_move( velX=0.1,std_time=5.0) 
            if find_placing_area(self.low_shelf_height):                
                return 'succ'
            omni_base.tiny_move( velX=0.1,std_time=5.0) 
            if find_placing_area(self.low_shelf_height):
                return 'succ'
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
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL",           Initial(),              transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'WAIT_PUSH_HAND',   
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
                                                                                         'succ': 'PICKUP',       
                                                                                         'tries': 'GOTO_PICKUP'})        
        smach.StateMachine.add("GOTO_SHELF",    Goto_shelf(),       transitions={'failed': 'GOTO_SHELF',    
                                                                                         'succ': 'SCAN_SHELF',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("GOTO_PLACE_SHELF",    Goto_place_shelf(),       transitions={'failed': 'GOTO_PLACE_SHELF',    
                                                                                         'succ': 'PLACE_SHELF',       
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
                                                                                         'tries': 'GOTO_PICKUP'})
        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction, goal_slots=['target_pose']),              
                        transitions={'preempted': 'END', 'succeeded': 'CHECK_GRASP', 'aborted': 'CHECK_GRASP'})
        
        ###################################################################################################################################################################
        


    outcome = sm.execute()
