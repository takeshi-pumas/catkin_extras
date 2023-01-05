#!/usr/bin/env python3

from smach_utils import * 


##### Define state INITIAL #####

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
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
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ=head.go() 
        
        #succ = True        
        if succ:
            return 'succ'
        else:
            return 'failed'

class GotoArea2(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):

        self.tries+=1
        rospy.loginfo('STATE : GOTO_AREA2')
        goal_x=df[df['child_id_frame']=='Location']['x'].values
        goal_y=df[df['child_id_frame']=='Location']['y'].values
        print (goal_x,goal_y)
        result_state=move_base(goal_x,goal_y,0.0,30)
        print (result_state)
        if self.tries==3:
            self.tries=0
            return 'tries'
        
        
        return 'succ'
        

class ScanFloor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):
        self.tries+=1
        if self.tries==3:return 'tries'
        rospy.loginfo('STATE : SCAN_FLOOR')
        hv= head.get_current_joint_values()
        hv[1]= -0.6
        succ= head.go(hv)
        res=segmentation_server.call()
        print (res)
        if(res.success):
            cents=strmsg_to_float(res.message)
            print ('cents',cents)       
            if len (cents)!=0:
                t= write_tf(cents[0,:],np.asarray((0,0,0,1)) , 'SM_Object' )
                print (t)        
                broadcaster.sendTransform(t)

                return 'succ'
            
        return 'failed'
        """try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            print ('trans',trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No TF FOUND')
        """
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
        smach.StateMachine.add("INITIAL",               Initial(),          transitions = {'failed':'INITIAL',          'succ':'SCAN_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("GOTO_AREA2",            GotoArea2(),          transitions = {'failed':'GOTO_AREA2',          'succ':'SCAN_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("SCAN_FLOOR",            ScanFloor(),          transitions = {'failed':'GOTO_START',          'succ':'PREGRASP_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("PREGRASP_FLOOR",        Pregrasp_Floor(),          transitions = {'failed':'PREGRASP_FLOOR',          'succ':'GRASP_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("GRASP_FLOOR",           Grasp_Floor(),          transitions = {'failed':'GRASP_FLOOR',          'succ':'POSTGRASP_FLOOR',           'tries':'END'}) 
        smach.StateMachine.add("POSTGRASP_FLOOR",       Postgrasp_Floor(),          transitions = {'failed':'POSTGRASP_FLOOR',          'succ':'GOTO_AREA2',           'tries':'END'}) 
        smach.StateMachine.add("GOTO_START",            GotoStart(),          transitions = {'failed':'GOTO_START',          'succ':'INITIAL',           'tries':'END'}) 
        
    outcome = sm.execute()

 
    
