#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction
from std_msgs.msg import Float32MultiArray



def categorize_objs(name):
    kitchen =['bowl','spatula','spoon', 'bowl','plate','f_cups','h_cups']
    tools=['extra_large_clamp','large_clamp','small_clamp','medium_clamp','adjustable_wrench','flat_screwdriver','phillips_screwdriver','wood_block']
    balls= ['softball','tennis_ball','a_mini_soccer_ball', 'racquetball', 'golf_ball', 'baseball'  ]
    fruits= ['apple','banana', 'lemon','pear','plum','orange','strawberry','peach']
    food =['chips_can','mustard_bottle','potted_meat_can','tomato_soup_can','tuna_fish_can','master_chef_can','sugar_box','pudding_box','cracker_box']
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
                                                                                                                #MUST BE SET

        print (f'Regions for Storing Groceries(Real Robot) {regions}')
        ##TO AVOID SMACH DYING IN CASE NO PLACING AREA IS FOUND, THere is a default that at least allows the test to continue
        x,y= 9.3 , -4.7  #np.mean(regions['shelves'], axis=0)
        z=0.4#self.mid_shelf_height=0.4 shelves heights must also be set on SCAN SHELF  state init sectoin.

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
        res = omni_base.move_base(known_location='pickup', time_out=200)
        print(res)
##################################################################First TIme Only go        
        if self.tries == 1: 
            talk('Navigating to, pickup')
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
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        self.tries += 1
        if self.tries >= 3:
            self.tries = 0            
            print ( "I could not find more objects ")
            return 'tries'
        if self.tries==1:
            head.set_joint_values([ 0.0, -0.5])
            talk('Scanning Table')
        
        global objs 
        print ('scanNING TABLE')
        rospy.sleep(5.0)                        
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        objects=detect_object_yolo('all',res)   
        if len (objects)!=0 :
            for i in range(len(res.poses)):
                #tf_man.getTF("head_rgbd_sensor_rgb_frame")
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                #print ('position,name',position,res.names[i].data[4:])
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

                #tf_man.pub_static_tf(pos= position, rot=[0,0,0,1], ref="head_rgbd_sensor_rgb_frame", point_name=res.names[i].data[4:] )   
                #rospy.sleep(0.3)
                #tf_man.change_ref_frame_tf(res.names[i].data[4:])
                #rospy.sleep(0.3)
                #pose , _=tf_man.getTF(res.names[i].data[4:])
                #if type (pose)!= bool:
                #    new_row = {'x': pose[0], 'y': pose[1], 'z': pose[2], 'obj_name': res.names[i].data[4:]}
                #    objs.loc[len(objs)] = new_row
        else:
            print('Objects list empty')
            return 'failed'
        
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('config_files')+'/regions'         
        #regions={'shelves':np.load(file_path+'/shelf_sim.npy'),'pickup':np.load(file_path+'/pickup_sim.npy')}
        regions={'shelves':np.load(file_path+'/shelves_region.npy'),'pickup':np.load(file_path+'/pickup_region.npy')}
        print (regions)
        def is_inside(x,y,z):return ((area_box[:,1].max() > y) and (area_box[:,1].min() < y)) and ((area_box[:,0].max() > x) and (area_box[0,0].min() < x)) and (0.65<z)  
        for name in regions:
            in_region=[]
            area_box=regions[name]
            for index, row in objs[['x','y','z']].iterrows():in_region.append(is_inside(row.x, row.y,row.z))
            objs[name]=pd.Series(in_region)
        cats=[]
        for name in objs['obj_name']:cats.append(categorize_objs(name))
        objs['category'] = cats    
        objs.dropna(inplace=True)
        objs.to_csv('/home/roboworks/Documents/objs.csv') # Debug DF --- REmove        
        print (objs)            
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
        pickup_objs=pickup_objs[pickup_objs['z']>0.69]#PICKUP AREA HEIGHT
        
        print ('pickup_objs',pickup_objs)        
        ix=np.argmin(np.linalg.norm(rob_pos-pickup_objs[['x','y','z']]  .values  , axis=1))
        name, cat=pickup_objs[['obj_name','category']].iloc[ix]
        print('closest',name , cat)
        talk ( f'closest object is {name}, of category {cat}, picking it up ')
        target_object= name  

        ##################################################
        pos, _ = tf_man.getTF(target_frame = target_object, ref_frame = 'odom')
        target_pose = Float32MultiArray()
        pos[2] += 0.03
        target_pose.data = pos
        userdata.target_pose = target_pose

        ###################
        head.set_named_target('neutral')
        rospy.sleep(0.5)
        

        clear_octo_client()
        
            

        return 'succ'
        
class Goto_shelf(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        #omni_base.tiny_move( velX=-0.2,std_time=4.2) 
        if self.tries == 1: talk('Navigating to, shelf')
        res = omni_base.move_base(known_location='shelf', time_out=200)
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
        if self.tries == 1: talk('Navigating to, shelf')
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
        high_shelf_place=[0.505,-0.586,0.0850,-0.934,0.0220,0.0]
        mid_shelf_place=[ 0.28,-1.6040,-0.0080,-0.0579,0.0019,0.0]
        low_shelf_place=[ 0.0457,-2.2625,0.0,0.7016,-0.0,0.0]
        
        ###########################################
        #placing_places=np.asarray(('placing_area_top_shelf1','placing_area_mid_shelf1','placing_area_low_shelf1'))
        po,_=tf_man.getTF('placing_area' )
        if po[2]> 0.6:placing_pose=high_shelf_place
        elif po[2]< 0.2:placing_pose=low_shelf_place
        else:placing_pose=mid_shelf_place
        
        ###########################################
        

        base_grasp_D(tf_name='placing_area',d_x=0.8, d_y=0.0,timeout=30)
        succ=arm.go(placing_pose)
        
        hand_grasp_D()  

        
        rospy.sleep(1.0)
        gripper.open()
        rospy.sleep(1.0)



        omni_base.tiny_move( velX=-0.3,std_time=4.2) 

 

        #base_grasp_D(tf_name='placing_area',d_x=0.6, d_y=0.0,timeout=30)
        rospy.sleep(1.0)
        gripper.steady()
        if succ:return'succ'
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
        omni_base.tiny_move( velX=-0.2,std_time=4.2) 

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
        objs.drop(objs[objs['obj_name'] == target_object].index, inplace=True)
        self.tries=0
        return'succ'    
                
                
                
                
                


        
#########################################################################################################   
class Scan_top_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
        self.top_shelf_height=0.88
        self.mid_shelf_height=0.4
        self.low_shelf_height=0.00
        



       
    def execute(self, userdata):
        global shelves_cats , objs_shelves
        self.tries+=1
        
        if "shelves_cats" in globals(): talk ( "I already scanned the shelf. Maybe dont scan it every time?")
        rospy.loginfo('State : Scanning_shelf')
        talk('Scanning shelf')
        print("talk('Scanning top shelf')")
        print (self.tries)
        ##################
        rospack = rospkg.RosPack()        
        
        
            
        file_path = rospack.get_path('config_files')+'/regions'         
        #regions={'shelves':np.load(file_path+'/shelf_sim.npy'),'pickup':np.load(file_path+'/pickup_sim.npy')}
        regions={'shelves':np.load(file_path+'/shelves_region.npy'),'pickup':np.load(file_path+'/pickup_region.npy')}
        print('area box',regions['shelves'],'###################################')
        if self.tries==3:
            print ('No category found placing at mid.... ')
            print ('GOTO PLACE SHELF MID')
            head.set_joint_values([0.0 , 0.0])
            av=arm.get_current_joint_values()
            av[0]=0.0            
            arm.go(av)
            rospy.sleep(1.0)
            omni_base.move_base(known_location='place_shelf', time_out=10)
            rospy.sleep(2.5)
            head.set_joint_values([0.0, -0.57])        
            rospy.sleep(2.5)
            find_placing_area(self.mid_shelf_height)

            self.tries=0
            return 'succ' # IMPLEMENT outcome->place top shelf
        
        if self.tries==1:
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
        def is_inside_top(x,y,z):return ((area_box[:,1].max() > y) and (area_box[:,1].min() < y)) and ((area_box[:,0].max() > x) and (area_box[0,0].min() < x))    and ((0.9 < z  )  )
        def is_inside_mid(x,y,z):return ((area_box[:,1].max() > y) and (area_box[:,1].min() < y)) and ((area_box[:,0].max() > x) and (area_box[0,0].min() < x))    and ((0.9 > z) and (0.3 < z  )  )  
        def is_inside_low(x,y,z):return ((area_box[:,1].max() > y) and (area_box[:,1].min() < y)) and ((area_box[:,0].max() > x) and (area_box[0,0].min() < x))    and ((0.3 > z  )  )
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
        a=objs_shelves[objs_shelves['low_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['low_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:
            talk('Category found in bottom shelf... Placing')
            print(f'LOW  shelf category {a.index[a.argmax()]}')
            shelves_cats['low'] =a.index[a.argmax()]
            if a.index[a.argmax()] == cat:
                print ('GOTO PLACE SHELF LOW')
                head.set_joint_values([0.0 , 0.0])
                av=arm.get_current_joint_values()
                av[0]=0.0            
                arm.go(av)
                rospy.sleep(1.0)
                omni_base.move_base(known_location='place_shelf', time_out=10)
                rospy.sleep(2.5)
                rospy.sleep(2.5)
                head.set_joint_values([0.0, -0.7])        
                rospy.sleep(2.5)
                find_placing_area(self.low_shelf_height)
                return 'succ' # IMPLEMENT outcome->place top shelf
            else : print (a.index[a.argmax()],cat, 'top_cat ,cat')
        

        a=objs_shelves[objs_shelves['mid_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['mid_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:
            print(f'MID  shelf category {a.index[a.argmax()]}')
            shelves_cats['mid'] =a.index[a.argmax()]
            if a.index[a.argmax()] == cat:
                talk('Category found in middle shelf... Placing')
                print ('GOTO PLACE SHELF MID')
                head.set_joint_values([0.0 , 0.0])
                av=arm.get_current_joint_values()
                av[0]=0.0            
                arm.go(av)
                rospy.sleep(1.0)

                omni_base.move_base(known_location='place_shelf', time_out=10)
                rospy.sleep(2.5)
                head.set_joint_values([0.0, -0.57])        
                rospy.sleep(2.5)
                find_placing_area(self.mid_shelf_height)
                return 'succ' # IMPLEMENT outcome->place top shelf
            else : print (a.index[a.argmax()],cat, 'top_cat ,cat')
        a=objs_shelves[objs_shelves['top_shelf']]['category'].value_counts()
        if 'other' in objs_shelves[objs_shelves['top_shelf']]['category'].values:a.drop('other',inplace=True)
        if len(a.values)!=0:
            print(f'TOP  shelf category {a.index[a.argmax()]}')
            shelves_cats['top'] =a.index[a.argmax()]
            if a.index[a.argmax()] == cat:
                talk('Category found in top shelf... Placing')
                print ('GOTO PLACE SHELF TOP')
                find_placing_area(self.top_shelf_height)
                return 'succ' # IMPLEMENT outcome->place top shelf
            else : print (a.index[a.argmax()],cat, 'top_cat ,cat')
       
        objs_shelves.to_csv('/home/roboworks/Documents/objs_shelves.csv') # Debug DF --- REmove
        print (f'#SHELVES CATS {shelves_cats}#')
        print ('################################')
        print ('################################')
        print ('################################')
        print ('################################')
        print ('################################')
        print ('################################')
        print ('################################')
        ################################
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
        
        smach.StateMachine.add("GOTO_PICKUP",    Goto_pickup(),       transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'SCAN_TABLE',       
                                                                                         'tries': 'GOTO_PICKUP',
                                                                                         'pickup':'PICKUP'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),       transitions={'failed': 'SCAN_TABLE',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'GOTO_PICKUP'})        
        smach.StateMachine.add("GOTO_SHELF",    Goto_shelf(),       transitions={'failed': 'GOTO_SHELF',    
                                                                                         'succ': 'SCAN_TOP_SHELF',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("GOTO_PLACE_SHELF",    Goto_place_shelf(),       transitions={'failed': 'GOTO_PLACE_SHELF',    
                                                                                         'succ': 'PLACE_SHELF',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("PLACE_SHELF",    Place_shelf(),       transitions={'failed': 'SCAN_TOP_SHELF',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("SCAN_TOP_SHELF",    Scan_top_shelf(),       transitions={'failed': 'SCAN_TOP_SHELF',    
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
