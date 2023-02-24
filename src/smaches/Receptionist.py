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


class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0

        
    def execute(self,userdata):

        
        rospy.loginfo('STATE : INITIAL')
        print('robot neutral pose')

        print('Try',self.tries,'of 3 attempts') 
        self.tries+=1
        if self.tries==3:
            return 'tries'
        #takeshi_talk_pub.publish(string_to_Voice('Gently, ...  push my hand to begin'))
        talk('Gently, ...  push my hand to begin')
        succ= wait_for_push_hand(100)
        #succ = True        #HEY THIS MUST BE COMMENTED DEBUGUING

        if succ:
            return 'succ'
        else:
            return 'failed'

class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):

        rospy.loginfo('State : SCAN_FACE')
        self.tries+=1        
        
        if self.tries==1:
            head.set_named_target('neutral')
            head.go() 
        if self.tries==2:
            hv= head.get_current_joint_values()
            hv[0]= -0.6
            hv[1]= 0.0
            head.go(hv) 
        if self.tries==3:
            hv= head.get_current_joint_values()
            hv[0]= 0.6
            hv[1]= 0.0
            head.go(hv) 
        if self.tries>=9:
            self.tries=0
            return'tries'
        
        #img=rgbd.get_image()  
        #req=RecognizeFaceRequest()
        #print ('Got  image with shape',img.shape)
        #strings=Strings()
        #string_msg= String()
        #string_msg.data='Anyone'
        #req.Ids.ids.append(string_msg)
        #img_msg=bridge.cv2_to_imgmsg(img)
        #req.in_.image_msgs.append(img_msg)
        #res= recognize_face(req)
        res=wait_for_face()##default 10 secs
        
        print('Cheking for faces')
        if res== None:
            return 'failed'
        if res != None:
            print('RESPONSE',res.Ids.ids[0]      )
            if res.Ids.ids[0].data == 'NO_FACE':
                print ('No face Found Keep scanning')
                return 'failed'
            else:
                print ('A face was found.')
                talk('I found you, I believe you are'+ res.Ids.ids[0].data)
                
                try:
                    trans,quat = tf_man.getTF(target_frame='head_rgbd_sensor_link')
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print ( 'No TF FOUND')
                trans= np.zeros(3)

                trans[2]+= res.Ds.data[0]##HALF DISTANCE

                #tf_man.pub_static_tf(pos=trans, point_name='Face', ref='head_rgbd_sensor_link')
                
                rospy.sleep(0.9)
                return 'succ'

class Detect_action(smach.State):
    
    
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
        #class_names=["wave_R","wave_L","neutral","drink","pointing"]
    def execute(self,userdata):

        rospy.loginfo('State : SCAN_FACE')
        self.tries+=1        
        
        if self.tries==1:
            #head.set_named_target('neutral')
            #head.go() 
            print("")
        if self.tries==2:
            hv= head.get_current_joint_values()
            hv[1]= hv[1]+0.2 
            head.go(hv) 

        if self.tries>=9:
            self.tries=0
            return'tries'
        rospy.sleep(1.0)
        talk('Please start pointing at the object ')
        print("Detecting action...")
        
        
        reqAct.visual=1
        reqAct.in_=1
        resAct=recognize_action(reqAct)
        print("Response:",resAct.i_out)

        if resAct.i_out == 4:
            talk('Pointing action detected')
            posM,_ = tf_man.getTF(target_frame='MANO',ref_frame='map')
            posC,_ = tf_man.getTF(target_frame='CODO',ref_frame='map')
            #print(posM)
            #print(posC)
            extraP=get_extrapolation(posM,posC,z=0)
            #print(extraP)
            tf_man.pub_static_tf(point_name='point_Obj', ref='map', pos= extraP)
            return 'succ'
        else:
            talk('I dont detect pointing, I will try again')
            return 'failed'
        
        

class Goto_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):


        rospy.loginfo('State : GOING TO OBJECT PUMAS NAV AND MAP')
        tf_man.change_ref_frame_tf(point_name='point_Obj', new_frame='map')
        #goal_pose, quat=listener.lookupTransform( 'map','Face', rospy.Time(0))
        obj_pose,_ = tf_man.getTF(target_frame='point_Obj',ref_frame='map')
        robot_pose,quat_r = tf_man.getTF(target_frame='base_link')
        yaw=tf.transformations.euler_from_quaternion(quat_r)[2]

        obj_pose=np.asarray(obj_pose)
        robot_pose=np.asarray(robot_pose)
        face_rob = obj_pose-robot_pose
        goal_pose= obj_pose-(face_rob*0.7/np.linalg.norm(face_rob))

        print('Object at:',goal_pose[0], goal_pose[1], goal_pose[2])   #X Y YAW AND TIMEOUT
        hcp = gaze.absolute(goal_pose[0], goal_pose[1], 0.0)
        move_base(goal_pose[0],goal_pose[1], 0.0 ,20  )   #X Y YAW AND TIMEOUT
        head.set_joint_value_target(hcp)
        #head.go()
        tf_man.pub_static_tf(pos=goal_pose, point_name='Goal_D', ref='map')
        return 'succ'

class Grasp_from_floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        #get close to goal
        #succ = False
        #THRESHOLD = 0.02
        #while not succ:
        #    trans,_ = tf_man.getTF(target_frame='Face', ref_frame='base_link')
        #    if type(trans) is not bool:
        #        eX, eY, _ = trans
        #        eX -= 0.35
        #        rospy.loginfo("Distance to goal: {:.3f}, {:.3f}".format(eX, eY))
        #        if abs(eY) < THRESHOLD:
        #            eY = 0
        #        if abs(eX) < THRESHOLD:
        #            eX = 0
        #        succ =  eX == 0 and eY == 0
        #        grasp_base.tiny_move(velX=0.2*eX, velY=-0.4*eY, std_time=0.2, MAX_VEL=0.3)
        #prepare arm to grasp pose
        clear_octo_client()
        floor_pose = [0.0,-2.47,0.0,0.86,-0.032,0.0]
        h_search = [0.0,-0.70]
        AR_starter.call()
        arm.set_joint_value_target(floor_pose)
        arm.go()
        head.set_joint_value_target(h_search)
        head.go()
        gripper.open()
        #grasp
        succ = False
        THRESHOLD = 0.02
        while not succ:
            trans,_ = tf_man.getTF(target_frame='ar_marker/704', ref_frame='hand_palm_link')
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
        clear_octo_client()
        arm.set_named_target('neutral')
        arm.go()
        head.set_named_target('neutral')
        head.go()
        talk("Done, Thanks for your attention")
        return 'succ'



def init(node_name):
    global scene, rgbd, head, whole_body, arm, gripper, goal, navclient, clear_octo_client, detect_waving_client, class_names, bridge, base_vel_pub, takeshi_talk_pub, order, navclient, recognize_face, pub_potfields_goal
    global tf_man, gripper, gaze, wrist, AR_starter, AR_stopper, grasp_base,recognize_action,reqAct
    rospy.init_node(node_name)
    head = moveit_commander.MoveGroupCommander('head')
    #gripper =  moveit_commander.MoveGroupCommander('gripper')
    #whole_body=moveit_commander.MoveGroupCommander('whole_body')
    arm =  moveit_commander.MoveGroupCommander('arm')

    #whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGBD()
    goal = MoveBaseGoal()
    tf_man = TF_MANAGER()
    gripper = GRIPPER()
    gaze = GAZE()
    wrist = WRIST_SENSOR()
    grasp_base=OMNIBASE()
    reqAct=RecognizeRequest()

    AR_starter = rospy.ServiceProxy('/marker/start_recognition',Empty)
    AR_stopper = rospy.ServiceProxy('/marker/stop_recognition',Empty)
    #############################################################################
    navclient=actionlib.SimpleActionClient('/navigate', NavigateAction)   ### PUMAS NAV ACTION LIB
    #navclient=actionlib.SimpleActionClient('/navigate_hmm', NavigateAction)   ### HMM NAV
    #navclient = #actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)########TOYOTA NAV
    ###############################################################################################
    #navclient = #actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    

    pub_potfields_goal = rospy.Publisher("/clicked_point",PointStamped,queue_size=10)
    clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
    bridge = CvBridge()
    #class_names=['002masterchefcan', '003crackerbox', '004sugarbox', '005tomatosoupcan', '006mustardbottle', '007tunafishcan', '008puddingbox', '009gelatinbox', '010pottedmeatcan', '011banana', '012strawberry', '013apple', '014lemon', '015peach', '016pear', '017orange', '018plum', '019pitcherbase', '021bleachcleanser', '022windexbottle', '024bowl', '025mug', '027skillet', '028skilletlid', '029plate', '030fork', '031spoon', '032knife', '033spatula', '035powerdrill', '036woodblock', '037scissors', '038padlock', '040largemarker', '042adjustablewrench', '043phillipsscrewdriver', '044flatscrewdriver', '048hammer', '050mediumclamp', '051largeclamp', '052extralargeclamp', '053minisoccerball', '054softball', '055baseball', '056tennisball', '057racquetball', '058golfball', '059chain', '061foambrick', '062dice', '063-amarbles', '063-bmarbles', '065-acups', '065-bcups', '065-ccups', '065-dcups', '065-ecups', '065-fcups', '065-gcups', '065-hcups', '065-icups', '065-jcups', '070-acoloredwoodblocks', '070-bcoloredwoodblocks', '071nineholepegtest', '072-atoyairplane', '073-alegoduplo', '073-blegoduplo', '073-clegoduplo', '073-dlegoduplo', '073-elegoduplo', '073-flegoduplo', '073-glegoduplo']
    #classify_client = rospy.ServiceProxy('/classify', Classify)
    print ('Waiting for face recog service')

    rospy.wait_for_service('recognize_act')    
    recognize_action = rospy.ServiceProxy('recognize_act', Recognize) 

    rospy.wait_for_service('recognize_face')
    recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)
    #train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)    
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    takeshi_talk_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    
    #sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_RECEPTIONIST')
    sis.start()


    with sm:
        #State machine for Restaurant
        #smach.StateMachine.add("INITIAL",           Initial(),          transitions = {'failed':'INITIAL',          'succ':'WAIT_PUSH_HAND',           'tries':'END'}) 
        #smach.StateMachine.add("WAIT_PUSH_HAND",   Wait_push_hand(),  transitions = {'failed':'WAIT_PUSH_HAND',  'succ':'SCAN_FACE',    'tries':'END'}) 
        
        smach.StateMachine.add("INITIAL",           Initial(),          transitions = {'failed':'INITIAL',          'succ':'SCAN_FACE',           'tries':'END'}) 
        smach.StateMachine.add("SCAN_FACE",   Scan_face(),  transitions = {'failed':'SCAN_FACE',  'succ':'DETECT_POINT',    'tries':'INITIAL'}) 

        #smach.StateMachine.add("DETECT_POINT",   Detect_action(),  transitions = {'failed':'DETECT_POINT',  'succ':'GOTO_OBJECT',    'tries':'INITIAL'}) 
        
        smach.StateMachine.add("GOTO_OBJECT",   Goto_object(),  transitions = {'failed':'GOTO_OBJECT',  'succ':'GRASP_FROM_FLOOR',    'tries':'INITIAL'}) 
        smach.StateMachine.add("GRASP_FROM_FLOOR",   Grasp_from_floor(),  transitions = {'failed':'GRASP_FROM_FLOOR',  'succ':'END',    'tries':'GRASP_FROM_FLOOR'}) 
    outcome = sm.execute()          