#!/usr/bin/env python3
# Author: Oscar
# ofc1227@tec.mx
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
        ############################
        arm = moveit_commander.MoveGroupCommander('arm')
        head.set_named_target('neutral')
        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)
        return 'succ'
#########################################################################################################
class Plan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'navigate','pick','place'
                                            ,'find_objects','follow','answer'],
                                            output_keys=['actions','params','command_id'],input_keys=['actions','params'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Execute PLAN')
        actions=userdata.actions
        print(f'Ollama plan to actions{actions} ')
        if len(actions)==0:
            print("I believe I have suceeded. Go back to initial and wait for command" )
            return 0
        else:
            current_action=camel_to_snake(actions[0]).lower()   ### LLMS Still do wierd things sometims
            
            if   current_action == 'navigate':return'navigate'
            elif current_action =='pick_object':return'pick'
            elif current_action =='place_object':return'place'
            elif current_action =='find_object':return'find_objects'
            elif current_action =='follow_person':return'follow'
            elif current_action =='answer_question':return'answer'
            elif current_action == 'identify_person': 
                userdata.command_id = userdata.params[0]  # Set the command_id parameter who to follow
                return 'id_person'  # This triggers the ID_PERSON state


        print(f'current action {current_action} wtf?')
        return 'succ'
        
#########################################################################################################
class Wait_command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],output_keys=['actions','params'])
        self.tries = 0

    def execute(self, userdata):
        #############################################################################################
        #return 'succ'## REMOVE  THIS IS ONLY FOR GAZEBO TESTING (no push hand simulated just skip)
        #############################################################################################  
        rospy.loginfo('STATE : Wait for Wait_command ')
        print('Waiting for Command')
        self.tries += 1
        print(f'Try {self.tries} of 4 attempts')
        if self.tries == 4:
            return 'tries'
        talk('Ready for GPSR, waiting for comand using QR')    
        #plan="plan=['FindObject(EndTable)', 'Navigate(EndTable)', 'IdentifyObject(largest_object)', 'ReportObjectAttribute(size)']"
        #succ = wait_for_qr(100) # TO DO
        succ=True
        req = ActionPlannerRequest()
        req.timeout=10
        command=action_planner_server(req)
        print(f'command.plan.data{command.plan.data}.')
        if command.plan.data== 'timed out':return 'failed'
        
        #plan="plan = [ Navigate(Desk),FollowPerson(Skyler),Navigate(dining_room)]"
        #plan= "plan = [ Navigate(end_table),FindObject(bowl),PickObject(bowl),Navigate(cupboard),PlaceObject(cupboard)]"
        #plan="plan = [ Navigate(cupboard),PlaceObject(cupboard)]"
        #command_msg= String()
        #command_msg.data=plan.split("=")[1].strip("[ ]")
        actions=[]
        params=[]
        for command in ("Navigate(cupboard),PlaceObject(cupboard)"):#command.plan.data.split(','):   ### TODO THIS IS SUPER BAD
            print (command)
            action = command.split('(')[0]  # Get the part before '('
            param = command.split('(')[1][:-1]  # Get the part inside '()'
            # Append to the respective arrays
            actions.append(action)
            params.append(param)
        userdata.actions=actions
        userdata.params=params
        if succ:
            return 'succ'
        else:
            return 'failed'
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
class Navigate(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params'])
        
    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        print(f'Navigating to,{userdata.params[0]} ')        
        #talk('Navigating to, pickup')        
        res = omni_base.move_base(known_location=userdata.params[0], time_out=40)
        res = omni_base.move_base(known_location=userdata.params[0], time_out=40)
        print(res)

        if res:
            userdata.actions.pop(0)
            userdata.params.pop(0)
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'
##########################################################
class Answer_question(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params'])
        
    def execute(self, userdata):

        rospy.loginfo('STATE : Answering Question')
        print(f'Question is,{userdata.params[0]} ')        
        #answer_question(userdata.params[0])
        res = True
        print(res)

        if res:
            userdata.actions.pop(0)
            userdata.params.pop(0)
            return 'succ'
        else:
            return 'failed'

class Find_objects(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params'])
    def execute(self, userdata):
        
        
        rospy.loginfo(f'State : Look for objects -> {userdata.params[0]}')   
        head.set_joint_values([ 0.0, -0.5])
        rospy.sleep(5.0)                       
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)        
        res      = classify_client(req)
        objects, poses=detect_object_yolo(userdata.params[0],res)   
        print(objects, poses,userdata.params[0])
        if len (objects)==0 :
            print(f'{userdata.params[0]} was not found, ask for human assistance')   
            talk(f'{userdata.params[0]} was not found, ask for human assistance')
            return 'failed' 

        else:
            print(f'{userdata.params[0]} found')
            talk(f'{userdata.params[0]} found')
            _,quat_base= tf_man.getTF('base_link')
            for i in range(len(res.poses)):
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                ##########################################################
                object_point = PointStamped()
                object_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
                object_point.point.x = position[0]
                object_point.point.y = position[1]
                object_point.point.z = position[2]
                position_map = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
                print ('position_map',position_map,'name' ,res.names[i].data[4:])
                #if is_inside(position_map.point.x,position_map.point.y,position_map.point.z): 
                tf_man.pub_static_tf(pos= [position_map.point.x,position_map.point.y,position_map.point.z], rot=quat_base, ref="map", point_name=res.names[i].data[4:] )
            userdata.actions.pop(0)
            userdata.params.pop(0)
            return 'succ'

class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed','tries'])
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
        
        
        in_region=[]
        for index, row in objs[['x','y','z']].iterrows():in_region.append(is_inside(row.x, row.y,row.z))
        objs['pickup']=pd.Series(in_region)
        cats=[]
        for name in objs['obj_name']:cats.append(categorize_objs(name))
        objs['category'] = cats    
        objs.dropna(inplace=True)
        self.tries=0 
        return 'succ'
#########################################################################################################

class Pickup(smach.State):   
    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose','mode'], input_keys =['actions','params'],outcomes=['succ', 'failed'])
        

    def execute(self, userdata):

        rospy.loginfo('STATE : PICKUP')
        rob_pos,_=tf_man.getTF('base_link')
        global target_object
        target_object= userdata.params[0]
        
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
        print (f'offset point for {target_object} is {offset_point }')
        ##################################
        #####################APPLY OFFSET
        object_point = PointStamped()
        object_point.header.frame_id = target_object              #"base_link"
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
        
        return 'succ'
        
#################################################################################################
class Place(smach.State):   

    def __init__(self):
        smach.State.__init__(self, output_keys=['target_pose','target_object','mode'], input_keys =['actions','params'],outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : PLACE')
######################################

        ##################################################
        head.set_joint_values([ 0.0, -0.5])
        rospy.sleep(1.0)                
        find_placing_area()
        pos, quat = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'odom')
        target_pose = Float32MultiArray()
        target_pose.data = pos
        offset_point=[0.0,0.1,0.1 ]
        ###################
        #####################APPLY OFFSET
        object_point = PointStamped()
        object_point.header.frame_id = "placing_area"    ##userdata.target_object#"base_link"
        object_point.point.x = offset_point[0]
        object_point.point.y = offset_point[1]
        object_point.point.z = offset_point[2]
        transformed_object_point = tfBuffer.transform(object_point, "map", timeout=rospy.Duration(1))
        tf_man.pub_static_tf(pos=[transformed_object_point.point.x,transformed_object_point.point.y,transformed_object_point.point.z],rot=quat,point_name='goal_for_place')
        ##################################################################3
        rospy.sleep(0.5)
        pos, quat = tf_man.getTF(target_frame = 'goal_for_place', ref_frame = 'odom')
        print (f"pos{pos}transformed_object_point.point.z{transformed_object_point.point.z}")
        pose_goal=np.concatenate((pos,quat))
        print (f'transalted_point-<{pose_goal}')    
        target_pose = Float32MultiArray()
        target_pose.data = pose_goal#pos
        userdata.target_pose = target_pose

        ###################
        head.set_named_target('neutral')
        string_msg = String()
        string_msg.data='above'                
        if transformed_object_point.point.z>=0.6:
            #av=arm.get_current_joint_values()
            #av[0]=+0.3
            #av[1]=-0.74
            #av[2]=0.0
            #arm.go(av) 
            string_msg.data='frontal'    
            #string_msg.data='pour'
        userdata.mode=string_msg             
        ###################################
        rospy.sleep(0.5)
        #clear octo client is recommended
        #clear_octo_client()               
        return 'succ'        
class Goto_shelf(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries', 'scanned_shelf'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Navigate to known location')
        
        
        
        if "shelves_cats" in globals() and len(shelves_cats)==3:
            ### IF ALREADY SCANNED JUMP HERE
            print (shelves_cats , cat)

            ####################################################################################3
            string_list=regions_df['shelves']['z'].split(',')
            top,mid,low=[list(map(float, s.split(','))) for s in string_list]
            top_shelf_height=top[0]  #0.98 #0.8
            mid_shelf_height=mid[0]  #0.41
            low_shelf_height=low[0]  #0.01    
            area_bo_x=regions_df['shelves'][['x_min','x_max']].values
            area_bo_y=regions_df['shelves'][['y_min','y_max']].values
            area_box=np.concatenate((area_bo_x,area_bo_y)).reshape((2,2)).T
            ###################################################################################################
            corresponding_key='low'  # fall back case
            for key, value in shelves_cats.items():
                print ('key',key)
                if value == cat:corresponding_key = key        
            print ('corresponding_key',corresponding_key)
            
            shelf_heights = {
            'top': top_shelf_height,
            'mid': mid_shelf_height,
            'low': low_shelf_height
            }
            print('here',objs_shelves[corresponding_key + '_shelf'])
            #objs_shelves.to_csv('/home/roboworks/Documents/objs.csv') 
            z_place = shelf_heights.get(corresponding_key, 0) + 0.1
            
            ################ PLACING AREA ESTIMATION FROM KNOWLEDGE DATA BASE
            y_range = np.arange(area_box[0,1]+0.05, area_box[1,1]-0.05, .06)
            x_range = np.arange(area_box[0,0]+0.12, area_box[1,0]-0.12, .06)
            print (f'ys = {y_range}, xs = {x_range}, areabox ={area_box}')
            grid = np.meshgrid(x_range, y_range)
            grid_points = np.vstack([grid[0].ravel(), grid[1].ravel()]).T        
            free_grid = grid_points.tolist()
            # Create a new list without the unwanted elements
            free_grid = [free_pt for free_pt in free_grid if all(
                np.linalg.norm(obj_pt - free_pt) >= 0.08 for obj_pt in objs_shelves[objs_shelves[corresponding_key + '_shelf'] == True][['x', 'y']].values)]
            free_grid=np.asarray((free_grid))
            
            
            if len (free_grid)==0:
                print ( 'no placing area found, using generic safe pose')
                talk ( f'no placing area found in {corresponding_key} shelf, using generic safe pose')
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
            res=omni_base.move_base(known_location='place_shelf', time_out=35)  #OPTIMAL VIEWING POSITION (PREKNOWN)          
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
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
    def execute(self, userdata):
        
        head.set_joint_values([0,0])
        rospy.loginfo('STATE : Placing in shelf')
        
        self.tries += 1
        print(f'shelves_cats{shelves_cats}, object picked up cat {cat}')
        ###########################################
        high_shelf_place=[0.669, -1.44, 0.292,  -0.01729,-0.338, 0.0]
        mid_shelf_place= [0.276, -1.581, 0.15, 0.0621, -0.1234, 0.0]
        low_shelf_place= [0.01, -2.09, 0.0, 0.455, -0.004, 0.0]
        low_shelf,mid_shelf,top_shelf,=False,False,False
        ###########################################
        string_list=regions_df['shelves']['z'].split(',')
        top,mid,low=[list(map(float, s.split(','))) for s in string_list]

        ############################################
        #placing_places=np.asarray(('placing_area_top_shelf1','placing_area_mid_shelf1','placing_area_low_shelf1'))
        po,_=tf_man.getTF('placing_area')
        if po[2]> top[0]:
            placing_pose=high_shelf_place
            top_shelf=True
            print('placing at top shelf')
            talk('placing top')

        elif po[2]< mid[0]:
            placing_pose=low_shelf_place
            low_shelf=True
            print('placing in bottom shelf')
            talk('placing low')
        else:
            placing_pose=mid_shelf_place
            mid_shelf=True
            print
            talk ('placing at middle shelf')
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
        #objs_shelves.to_csv('/home/roboworks/Documents/objs.csv') 

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
        head.set_joint_values([0,0])
        omni_base.tiny_move( velX=-0.3,std_time=10.0) 
        arm.set_named_target('go')
        gripper.steady()
        arm.go()
        rospy.sleep(1.5)


 

        #base_grasp_D(tf_name='placing_area',d_x=0.6, d_y=0.0,timeout=30)
        #rospy.sleep(1.0)
        #gripper.steady()
        if succ:
            self.tries=0
            return'succ'
        return'failed'
        
class Post_place(smach.State):   
    def __init__(self):
        smach.State.__init__(self,input_keys=['actions','params'] ,output_keys=['target_pose','actions','params'],outcomes=['succ', 'failed'])
        self.tries=0

    def execute(self, userdata):        
        userdata.actions.pop(0)
        userdata.params.pop(0)
        head.set_named_target('neutral')
        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)
        return 'succ'
        return'succ'                
#########################################################################################################
class Check_grasp(smach.State):   
    def __init__(self):
        smach.State.__init__(self,input_keys=['actions','params'] ,output_keys=['target_pose','actions','params'],outcomes=['succ', 'failed'])
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
                            
                test_pt=np.asarray((position_map.point.x,position_map.point.y,position_map.point.z))
                #print (np.linalg.norm(pose_target-test_pt))
                if check_if_grasped(pose_target,test_pt):
                    print (f'Centroid found in area {test_pt}, obj_name: {res.names[i].data[4:]}')
                    print ('Grasping May have failed')
                    talk ('Grasping May have failed')
                    return 'failed'
        
        userdata.actions.pop(0)
        userdata.params.pop(0)
        return'succ'                
        
#########################################################################################################   
class Scan_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
       
    def execute(self, userdata):
        """
        Find a placing area

        Args:
            userdata: The user data for the task.

        Returns:
            The result of the execution.
        """
        global shelves_cats, objs_shelves
        self.tries += 1

        # Rest of the code...
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
        shelf_quat=[regions_df['shelves']['quat_w'],regions_df['shelves']['quat_x'],regions_df['shelves']['quat_y'],regions_df['shelves']['quat_z']]
        print (f'shelf_quat{shelf_quat}')
        string_list=regions_df['shelves']['z'].split(',')

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
        ####################################
        if "shelves_cats" in globals() and len(shelves_cats)==3:###SHELF ALREADY SCANNED
            talk ( "I already scanned the shelf.")            
            print (shelves_cats , cat,"I already scanned the shelf. Maybe dont scan it every time?")
            corresponding_key='low'
            for key, value in shelves_cats.items():
                print (key, value)
                if value == cat:
                    corresponding_key = key  #Iterate over shelves cats and decide
                    print (corresponding_key,key)
            print ('corresponding_key',corresponding_key )
            
        else:
    ###########################################################################################            
            rospy.loginfo('State : Scanning_shelf')
            clear_octo_client()
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
                head.set_joint_values([-np.pi/2 , -0.9])        
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

            def is_inside_top(x,y,z):return ((area_box[1,1] +0.1 > y) and (area_box[0,1]-0.1 < y)) and ((area_box[1,0] +0.1  > x) and (area_box[0,0]-0.1  < x))    and (top_shelf_height < z )  
            def is_inside_mid(x,y,z):return ((area_box[1,1] +0.1 > y) and (area_box[0,1]-0.1 < y)) and ((area_box[1,0] +0.1  > x) and (area_box[0,0]-0.1  < x))    and ((0.9*top_shelf_height > z) and (mid_shelf_height < z  )  )  
            def is_inside_low(x,y,z):return ((area_box[1,1] +0.1 > y) and (area_box[0,1]-0.1 < y)) and ((area_box[1,0] +0.1  > x) and (area_box[0,0]-0.1  < x))    and ((0.9*mid_shelf_height > z  )  )
            #<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            for i in range (5):
                image= cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR)
                img_msg  = bridge.cv2_to_imgmsg(image)
                req      = classify_client.request_class()
                req.in_.image_msgs.append(img_msg)
                res      = classify_client(req)
                debug_image=   cv2.cvtColor(bridge.imgmsg_to_cv2(res.debug_image.image_msgs[0]), cv2.COLOR_RGB2BGR)
                objects=detect_object_yolo('all',res)  # list of objects detected objects
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
                        if (is_inside_top(x,y,z)    or                is_inside_mid(x,y,z)  or               is_inside_low(x,y,z)):  tf_man.pub_static_tf(pos=[x,y,z],rot=shelf_quat
                                                                                                                                        ,point_name=res.names[i].data[4:]) ### IF a real placing area is found this tf will be updated
                        new_row = {'x': position_map.point.x, 'y': position_map.point.y, 'z': position_map.point.z, 'obj_name': res.names[i].data[4:]}
                        print (objs_shelves,new_row, top_shelf_height)
                        objs_shelves.loc[len(objs_shelves)] = new_row
                else:
                    print ('no objs')
                    return 'failed'
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
                missing_shelves=[]
                for shelf in ['top', 'low', 'mid']:
                    a=objs_shelves[objs_shelves[f'{shelf}_shelf']]['category'].value_counts()
                    if 'other' in a:
                        a.drop('other',inplace=True)
                    if not a.empty:
                        print(f'{shelf.upper()} shelf category {a.index[a.argmax()]}')
                        shelves_cats[shelf] =a.index[a.argmax()]
                    else:
                        missing_shelves.append(shelf)
                if len (missing_shelves)==0:break
                else:
                    if (missing_shelves[0]=='top'): 
                        talk( 'top shelf missing, scanning again')
                        head.set_joint_values([-np.pi/2 , -0.3])    
                    elif (missing_shelves[0]=='mid'): 
                        head.set_joint_values([-np.pi/2 , -0.7])  
                        talk( 'mid shelf missing, scanning again')  
                    elif (missing_shelves[0]=='low'): 
                        head.set_joint_values([-np.pi/2 , -1.0])    
                        talk( 'bottom shelf missing, scanning again')
                    rospy.sleep(1.0)

        ### IF ALREADY SCANNED JUMP HERE
        print (f'shelves_cats , cat{shelves_cats , cat} missing shelves {missing_shelves}')
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
        talk ( f' I will place object in {corresponding_key} shelf')
        #objs_shelves.to_csv('/home/roboworks/Documents/objs.csv') 
        z_place = shelf_heights.get(corresponding_key, 0) + 0.1
        ################ PLACING AREA ESTIMATION FROM KNOWLEDGE DATA BASE
        print (area_box)
        y_range = np.arange(area_box[0,1]+0.05, area_box[1,1]-0.05, .06)
        x_range = np.arange(area_box[0,0]+0.12, area_box[1,0]-0.12, .06)
        print (f'ys = {y_range}, xs = {x_range}')
        grid = np.meshgrid(x_range, y_range)
        grid_points = np.vstack([grid[0].ravel(), grid[1].ravel()]).T        
        free_grid = grid_points.tolist()
        # Create a new list without the unwanted elements
        free_grid = [free_pt for free_pt in free_grid if all(
            np.linalg.norm(obj_pt - free_pt) >= 0.08 for obj_pt in objs_shelves[objs_shelves[corresponding_key + '_shelf'] == True][['x', 'y']].values)]
        free_grid=np.asarray((free_grid))        
        if len (free_grid)==0:
            print ( 'no placing area found, using generic safe pose')
            talk ( f'no placing area found in {corresponding_key}, using generic safe pose')
            x,y= np.mean(area_box, axis=0)
            z=0.44#self.mid_shelf_height=0.4 shelves heights must also be set on SCAN SHELF  state init section.
            tf_man.pub_static_tf(pos=[x,y,z],point_name='placing_area') ### IF a real placing area is found this tf will be updated
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
    sm.userdata.first = True
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL"            ,   Initial(),          transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'WAIT_COMMAND',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_COMMAND"       ,   Wait_command(),       transitions={'failed': 'WAIT_COMMAND',    
                                                                                         'succ': 'PLAN',       
                                                                                         'tries': 'END'})
        smach.StateMachine.add("PLAN"               ,   Plan(),        transitions={'failed': 'PLAN', 
                                                                                        'succ': 'END',
                                                                                        'pick':'PICKUP',
                                                                                        'place':'PLACE',  
                                                                                        'find_objects':'FIND_OBJECTS',  
                                                                                        'navigate': 'NAVIGATE',
                                                                                        'follow'  :'FOLLOW_PERSON',
                                                                                        'answer'  :'ANSWER_QUESTION'
                                                                                         })
      

        smach.StateMachine.add("FIND_OBJECTS"           ,   Find_objects(),     transitions={'failed': 'FIND_OBJECTS',    
                                                                                         'succ': 'PLAN',
                                                                                         })        
        smach.StateMachine.add("NAVIGATE"               ,   Navigate(),           transitions={'failed': 'NAVIGATE',    
                                                                                         'succ': 'PLAN', 
                                                                                         })
        smach.StateMachine.add("ANSWER_QUESTION"        ,   Answer_question(),     transitions={'failed': 'ANSWER_QUESTION',    
                                                                                         'succ': 'PLAN', 
                                                                                         })
        
        smach.StateMachine.add("PICKUP",    Pickup(),                   transitions={'failed': 'PICKUP',    
                                                                                         'succ': 'GRASP_GOAL'})
        smach.StateMachine.add("PLACE",    Place(),                     transitions={'failed': 'PLACE',    
                                                                                         'succ': 'PLACE_GOAL',
                                                                                         'tries': 'PLAN'})
        
        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction, goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),                      
                                transitions={'preempted': 'PLAN', 'succeeded': 'CHECK_GRASP', 'aborted': 'PLAN'})
       
        smach.StateMachine.add("PLACE_GOAL", SimpleActionState('place_server', GraspAction, goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),                    
                                transitions={'preempted': 'PLACE', 'succeeded': 'POST_PLACE', 'aborted': 'PLACE'})
        
        smach.StateMachine.add("ID_PERSON", SimpleActionState('id_server', IdentifyPersonAction, goal_slots={'command_id': 'command_id'}),
                               transitions={'aborted': 'ID_PERSON','succeeded': 'PLAN' ,'preempted': 'PLAN' })
        
        
        smach.StateMachine.add("FOLLOW_PERSON", SimpleActionState('follow_server', FollowAction), transitions={'aborted': 'FOLLOW_PERSON',        
                                                                                                            'succeeded': 'PLAN' ,   
                                                                                                            'preempted': 'PLAN' })
        
        
                        
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'WAIT_COMMAND',       
                                                                                         'tries': 'END'})

        smach.StateMachine.add("WAIT_DOOR",    Wait_door_opened(),      transitions={'failed': 'WAIT_DOOR',    
                                                                                         'succ': 'WAIT_COMMAND'})

        
        
        smach.StateMachine.add("CHECK_GRASP",    Check_grasp(),         transitions={'failed': 'PLAN',    
                                                                                         'succ': 'PLAN',
                                                                                         })
        ###################################################################################################################################################################
        smach.StateMachine.add("POST_PLACE",    Post_place(),         transitions={'failed': 'PLAN',    
                                                                                         'succ': 'PLAN',
                                                                                         })
        ###################################################################################################################################################################
        


    outcome = sm.execute()
