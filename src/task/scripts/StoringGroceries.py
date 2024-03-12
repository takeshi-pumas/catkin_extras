#!/usr/bin/env python3
from smach_utils2 import *
from smach_ros import SimpleActionState


def categorize_objs(name):
    kitchen =['bowl','spatula','spoon', 'bowl','plate']
    tools=['extra_large_clamp','large_clamp','small_clamp','medium_clamp','adjustable_wrench','flat_screwdriver','phillips_screwdriver','wood_block']
    balls= ['softball','tennis_ball','a_mini_soccer_ball']
    fruits= ['apple','banana', 'lemon','pear']
    food =['chips_can','mustard_bottle','potted_meat_can','tomato_soup_can','tuna_fish_can','master_chef_can']
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
        
        #READ YAML ROOMS XYS   objs csv, known initial object locations
        
        global arm ,  hand_rgb , objs
        hand_rgb = HAND_RGB()
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('config_files') 
        objs = pd.read_csv (file_path+'/objects.csv')
        objs=objs.drop(columns='Unnamed: 0')

        arm = moveit_commander.MoveGroupCommander('arm')
        head.set_named_target('neutral')

        rospy.sleep(0.8)
        arm.set_named_target('go')
        arm.go()
        rospy.sleep(0.3)

        #gripper.open()
        #rospy.sleep(0.3)

        #gripper.close()
        #rospy.sleep(0.3)

        
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
        objects=detect_object_yolo('all',res)   
        if len (objects)!=0 :
            for i in range(len(res.poses)):
                tf_man.getTF("head_rgbd_sensor_rgb_frame")
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                tf_man.pub_static_tf(pos= position, rot=[0,0,0,1], ref="head_rgbd_sensor_rgb_frame", point_name=res.names[i].data[4:] )   
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(res.names[i].data[4:])
                rospy.sleep(0.3)
                pose , _=tf_man.getTF(res.names[i].data[4:])
                new_row = {'x': pose[0], 'y': pose[1], 'z': pose[2], 'obj_name': res.names[i].data[4:]}
                objs.loc[len(objs)] = new_row
        else:
            print('Objects list empty')
            return 'failed'
        print(objs)
        
        return 'succ'
#########################################################################################################
    
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
        if self.tries == 1: talk('Navigating to, shelf')
        res = omni_base.move_base(known_location='shelf', time_out=200)
        print(res)

        if res:
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'

#########################################################################################################
class Scan_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0
       
    def execute(self, userdata):
        rospy.loginfo('State : Scanning_table')
        talk('Scanning shelf')
        print("talk('Scanning shelf')")
        self.tries += 1
        if self.tries >= 4:
            self.tries = 0
            ####REMOVE THIS LINE; ONLY FOR DEBGNG PURPOSES
            regions={'shelves':np.load('/home/roboworks/Documents/shelves_region.npy'),'pickup':np.load('/home/roboworks/Documents/pickup_region.npy')}
            def is_inside(x,y):return ((area_box[1,1] > y) and (area_box[0,1] < y)) and ((area_box[1,0] > x) and (area_box[0,0] < x)) 
            for name in regions:
                in_region=[]
                area_box=regions[name]
                for index, row in objs[['x','y']].iterrows():in_region.append(is_inside(row.x, row.y))
                objs[name]=pd.Series(in_region)
            cats=[]
            for name in objs['obj_name']:cats.append(categorize_objs(name))
            objs['category'] = cats   
            print ('################################')
            print (objs)
            objs.to_csv('/home/roboworks/Documents/objs.csv')
            print ('################################')
            print ('################################')
            print ('################################')
            print ('################################')
            print ('################################')
            print ('################################')
            print ('################################')
            print ('################################')






            ################################
            return 'tries'
        if self.tries==1:
            head.set_named_target('neutral')
            av=arm.get_current_joint_values()
            av[0]=0.5
            av[1]=-0.5
            arm.go(av)
            head.set_joint_values([-np.pi/2 , -0.5])
            
            
            
        if self.tries==2:
            av=arm.get_current_joint_values()
            av[0]=0.05
            arm.go(av)
                
        if self.tries==3:
            head.set_joint_values([-np.pi/2 , -0.8])
            av=arm.get_current_joint_values()
            av[0]=0.0
            av[1]=-0.5
            arm.go(av)
        rospy.sleep(1.3)                      
        img_msg  = bridge.cv2_to_imgmsg(rgbd.get_image())
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)
        res      = classify_client(req)
        objects=detect_object_yolo('all',res)   
        if len (objects)!=0 :
            for i in range(len(res.poses)):
                tf_man.getTF("head_rgbd_sensor_rgb_frame")
                position = [res.poses[i].position.x ,res.poses[i].position.y,res.poses[i].position.z]
                tf_man.pub_static_tf(pos= position, rot=[0,0,0,1], ref="head_rgbd_sensor_rgb_frame", point_name=res.names[i].data[4:] )   
                rospy.sleep(0.3)
                tf_man.change_ref_frame_tf(res.names[i].data[4:])
                rospy.sleep(0.3)
                pose , _=tf_man.getTF(res.names[i].data[4:])
                new_row = {'x': pose[0], 'y': pose[1], 'z': pose[2], 'obj_name': res.names[i].data[4:]}
                objs.loc[len(objs)] = new_row
        else:
            print('Objects list empty')
            return 'failed'
        print(objs)
        
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
                                                                                         'succ': 'WAIT_PUSH_HAND',   
                                                                                         'tries': 'END'})
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),       transitions={'failed': 'WAIT_PUSH_HAND',    
                                                                                         'succ': 'GOTO_PICKUP',       
                                                                                         'tries': 'END'})
        
        smach.StateMachine.add("GOTO_PICKUP",    Goto_pickup(),       transitions={'failed': 'GOTO_PICKUP',    
                                                                                         'succ': 'SCAN_TABLE',       
                                                                                         'tries': 'GOTO_PICKUP'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),       transitions={'failed': 'SCAN_TABLE',    
                                                                                         'succ': 'GOTO_SHELF',       
                                                                                         'tries': 'GOTO_PICKUP'})
        
        smach.StateMachine.add("GOTO_SHELF",    Goto_shelf(),       transitions={'failed': 'GOTO_SHELF',    
                                                                                         'succ': 'SCAN_SHELF',       
                                                                                         'tries': 'GOTO_SHELF'})
        smach.StateMachine.add("SCAN_SHELF",    Scan_shelf(),       transitions={'failed': 'SCAN_SHELF',    
                                                                                         'succ': 'SCAN_SHELF',       
                                                                                         'tries': 'END'})
        
        ###################################################################################################################################################################
        


    outcome = sm.execute()
