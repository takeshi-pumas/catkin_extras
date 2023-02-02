#!/usr/bin/env python3

from smach_utils import * 



##### Define state INITIAL #####

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0

        
    def execute(self,userdata):

        
        rospy.loginfo('STATE : INITIAL')
        print('robot neutral pose')

        print('Try',self.tries,'of 5 attempts') 
        self.tries+=1
        if self.tries==3:
            return 'tries'
        
        #clear_octo_client()
        
        #scene.remove_world_object()
        #Takeshi neutral
        head.set_named_target('neutral')
        succ=head.go() 
        
        arm.set_named_target('go')
        succ=arm.go()

        #succ = True        
        if succ:
            return 'succ'
        else:
            return 'failed'

class GotoArea(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    
        
    def execute(self,userdata):

        self.tries+=1
        rospy.loginfo('STATE : GOTO_AREA')
        goal_x=df[df['child_id_frame']=='Dining_room']['x'].values
        goal_y=df[df['child_id_frame']=='Dining_room']['y'].values
        print (goal_x,goal_y,"GOAL------------------")
        result_state=move_base(goal_x,goal_y,0.0,30)
        print (result_state)
        if self.tries==5:
            self.tries=0
            return 'tries'
        #rospy.publisher('goal_location', std_msgs/String)
        
        return 'succ'
        

class ScanFloor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0

        
    def execute(self,userdata):
        self.tries+=1
        if self.tries==4:return 'tries'

        rospy.loginfo('STATE : SCAN_FLOOR')
        hv= head.get_current_joint_values()
        if self.tries==1:
            hv[1]= -1.0
            hv[0]=  0.0
        if self.tries==2:
            hv[1]= -1.0
            hv[0]= -0.7
        if self.tries==3:
            hv[1]= -1.0
            hv[0]=  0.7
        
                

        head.go(hv)
        res=segmentation_server.call()
        print (res.poses.data)
        if len (res.poses.data)!=0:
            cents=np.asarray(res.poses.data).reshape((int(len (res.poses.data)/3),3 )   )
            print ('cents',cents)
            for i , cent in enumerate(cents):
                print(cent)
                tf_man.pub_static_tf(pos=cent, point_name='SM_Object'+str(i), ref='head_rgbd_sensor_link')
                tf_man.change_ref_frame_tf(point_name='SM_Object'+str(i), new_frame='map')
            trans,quat = tf_man.getTF(target_frame='SM_Object0')
            goal_pose,yaw=move_D(trans,0.75)    
            tf_man.pub_static_tf(pos=goal_pose,point_name='Goal_D')
            move_base(goal_pose[0],goal_pose[1], yaw ,10  ) 
            head.set_named_target('neutral')
            head.go()
            return 'succ'
        return'failed'
                


class Pregrasp_Floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:return 'tries'
        rospy.loginfo('STATE : PREGRASP_FLOOR')
        
        arm_grasp_floor = [0.1,-2.6,-0.26,0.701,0.10,0.0]
        arm.set_joint_value_target(arm_grasp_floor)
        succ=arm.go()


        #move_abs(.03,.03,0,2)
        
        


        try:
            trans = tfBuffer.lookup_transform('map', 'SM_Object', rospy.Time())
            obj_pose,obj_quat=read_tf(trans)
            print ('pose hand',obj_pose, obj_quat)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No TF FOUND')
            return 'failed'

        try:
            trans = tfBuffer.lookup_transform('map', 'hand_palm_link', rospy.Time())
            hand_pose,hand_quat=read_tf(trans)
            print ('pose hand',hand_pose, hand_quat)

            
            vec_vel=obj_pose-hand_pose

            print ("vec_vel=obj_pose-hand_pose", vec_vel)
            open_gripper()
            move_abs(.3*vec_vel[0],0.3*vec_vel[1],0,1)
            return 'succ'
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No TF FOUND')
            return 'failed'

       
class Grasp_Floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):
        self.tries+=1
        if self.tries==100:return 'tries'
        rospy.loginfo('STATE : GRASP_FLOOR')
        clear_octo_client()

        try:
            trans = tfBuffer.lookup_transform('map', 'SM_Object', rospy.Time())
            obj_pose,obj_quat=read_tf(trans)
            print ('pose object',obj_pose   )
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No TF FOUND')
            return 'failed'

        try:
            trans = tfBuffer.lookup_transform('map', 'hand_palm_link', rospy.Time())
            hand_pose,hand_quat=read_tf(trans)
            print ('pose hand',hand_pose, hand_quat)
            vec_vel=obj_pose-hand_pose
            print ("vec_vel=obj_pose-hand_pose", vec_vel)
            move_abs(0.1*vec_vel[0],0.3*vec_vel[1],0,2)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No TF FOUND')
            return 'failed'

        
        if np.linalg.norm(vec_vel[:2])< 0.1:
            self.tries=0
            clear_octo_client()
            close_gripper()
            return 'succ'
        return'failed'


        
        
class Postgrasp_Floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        

    def execute(self,userdata):

        self.tries+=1
        rospy.loginfo('STATE : POSTGRASP_FLOOR')
        close_gripper()
        arm.set_named_target('go')
        succ=arm.go()
        if self.tries==15:
            self.tries=0
            return 'tries'
        
        if succ:return 'succ'
        clear_octo_client()

        av=arm.get_current_joint_values()
        av[0]+=0.2
        arm.go(av)
        return 'failed'

class GotoStart(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        

    def execute(self,userdata):

        self.tries+=1
        rospy.loginfo('STATE : GOTO_START')
        result_state=move_base(0.1,0.1,0.0,60)
        print (result_state)
        if self.tries==5:
            self.tries=0
            return 'tries'
        
            
        return 'succ'
        



class Grasp_from_floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        #get close to goal
        #prepare arm to grasp pose
        clear_octo_client()
        floor_pose = [0.0,-2.47,0.0,0.86,-0.032,0.0]
        h_search = [0.0,-0.70]
        #AR_starter.call()
        arm.set_joint_value_target(floor_pose)
        arm.go()
        head.set_joint_value_target(h_search)
        head.go()
        gripper.open()
        #grasp
        succ = False
        THRESHOLD = 0.02
        while not succ:
            trans,_ = tf_man.getTF(target_frame='SM_Object0', ref_frame='hand_palm_link')
            if type(trans) is not bool:
                _, eY, eX = trans
                eX -= 0.05
                rospy.loginfo("Distance to goal: {:.3f}, {:.3f}".format(eX, eY))
                if abs(eY) < THRESHOLD:
                    eY = 0
                if abs(eX) < THRESHOLD:
                    eX = 0
                succ =  eX == 0 and eY == 0
                grasp_base.tiny_move(velX=0.2*eX, velY=-0.4*eY, std_time=0.2, MAX_VEL=0.3)
        gripper.close()
        grasp_base.tiny_move(velX=-0.5, std_time=0.5, MAX_VEL=0.1)
        rospy.sleep(0.5)
        arm.set_named_target('neutral')
        arm.go()
        head.set_named_target('neutral')
        head.go()
        talk("Done, Thanks for your attention")
        return 'succ'



def init(node_name):
    print ('smach ready')

    
    
    
    
#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    
    #sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_MOVE_SCAN_GRASP')
    sis.start()


    with sm:
        #State machine for Restaurant
        smach.StateMachine.add("SCAN_FLOOR",            ScanFloor(),          transitions = {'failed':'SCAN_FLOOR',   'succ':'GRASP_FROM_FLOOR', 'tries':'END'}) 
        smach.StateMachine.add("INITIAL",               Initial(),          transitions = {'failed':'INITIAL',          'succ':'GOTO_AREA',           'tries':'END'}) 
        smach.StateMachine.add("GOTO_AREA",            GotoArea(),          transitions = {'failed':'GOTO_AREA',          'succ':'SCAN_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("PREGRASP_FLOOR",        Pregrasp_Floor(),          transitions = {'failed':'PREGRASP_FLOOR',          'succ':'GRASP_FROM_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("GRASP_FROM_FLOOR",           Grasp_from_floor(),          transitions = {'failed':'GRASP_FROM_FLOOR',          'succ':'POSTGRASP_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("POSTGRASP_FLOOR",       Postgrasp_Floor(),          transitions = {'failed':'POSTGRASP_FLOOR',          'succ':'GOTO_AREA',           'tries':'END'}) 
        smach.StateMachine.add("GOTO_START",            GotoStart(),          transitions = {'failed':'GOTO_START',          'succ':'INITIAL',           'tries':'END'}) 
        
    outcome = sm.execute()

 
    
