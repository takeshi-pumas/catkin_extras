#!/usr/bin/env python3
from utils_takeshi import *

from smach_utils_carry import *


from std_msgs.msg import Bool 
from geometry_msgs.msg import Twist , PointStamped


    



#---------------------------------------------------
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):

        rospy.loginfo('STATE : INITIAL')
        print('Robot neutral pose')
        self.tries+=1
        print('Try',self.tries,'of 5 attempts') 
        if self.tries == 3:
            return 'tries'
        
        #Takeshi neutral
        head.set_named_target('neutral')
        #print('head listo')
        #brazo.set_named_target('go')
        #print('arm listo')
        rospy.sleep(0.8)
        return 'succ'

#---------------------------------------------------        
class Wait_push_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
        
    def execute(self,userdata):
        rospy.loginfo('STATE : INITIAL')
        print('robot neutral pose')

        print('Try',self.tries,'of 3 attempts') 
        self.tries+=1
        if self.tries==4:
            return 'tries'
        gripper.steady()
        brazo.set_named_target()
        talk('Gently, ...  push my hand to begin')
        succ= wait_for_push_hand(100)

        if succ:
            return 'succ'
        else:
            return 'failed'

#---------------------------------------------------
class Scan_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):

        rospy.loginfo('State : SCAN_FACE')
        self.tries+=1        
        
        if self.tries==1:
            hv = [0.0, 0.0]
        elif self.tries==2:
            hv = [-0.6, 0.0]
        elif self.tries==3:
            hv = [0.6, 0.0]
        elif self.tries>=9:
            self.tries=0
            return'tries'
        head.set_joint_values(hv)
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

#---------------------------------------------------
class Detect_action(smach.State):
    
    
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
        #class_names=["wave_R","wave_L","neutral","drink","pointing"]
    def execute(self,userdata):

        rospy.loginfo('State : Detect_action')
        self.tries+=1        

        pointing_h=['left','right']
        head.set_named_target('neutral')        
        rospy.sleep(1.0)
        hv= head.get_joint_values()
        hv[1]= hv[1]+0.15
        head.set_joint_values(hv)
        rospy.sleep(1.0)
        talk('Please start pointing at the object ')
        print("Detecting action...")
        
        # reqAct.visual --> 1 para visualizar imagen con openpose
        reqAct.visual=0
        # reqAct.in_ --> 3  para detectar pointing sin usar hmm, puro uso de vectores
        reqAct.in_=3
        resAct=recognize_action(reqAct)
        talk('I detect the '+pointing_h[resAct.i_out]+' hand pointing, is it correct?')
        """
        # PARA CONFIRMAR SI ESTA APUNTANDO BIEN, TIENE ERRORES
        rospy.sleep(0.1)
        res_sp=speech_recog_server()
        try:
            res2 = rospy.wait_for_message( '/recognizedSpeech',RecognizedSpeech, timeout = 5.0)
            print ("POCKET",res2.hypothesis[0],type(res2.hypothesis))
            name = res_sp.data = res2.hypothesis[0]
            print("NAME,",name)
        except Exception:   
            return 'failed'
        print(name)
        if name=="yes" or name=="Yes":
            # Retorna la coordenada xyz de mapa de la extrapolacion calculada al apuntar
            #print(resAct)
            obj_xyz=resAct.d_xyz.data
            print(obj_xyz)
            tf_man.pub_static_tf(pos=obj_xyz,point_name='object_gaze',ref='map')
            
            return 'succ'
        else:
            talk('Ok, I will try again')
            rospy.sleep(0.8)
            return 'failed'
        """
        obj_xyz=resAct.d_xyz.data
        print(obj_xyz)
        tf_man.pub_static_tf(pos=obj_xyz,point_name='object_gaze',ref='map')
        
        return 'succ'

#---------------------------------------------------
class Segment_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0

        
    def execute(self,userdata):
        self.tries+=1
        if self.tries==4:return 'tries'

        rospy.loginfo('STATE : Segment_object')
        hv= head.get_joint_values()

        talk('Segmenting object, please wait')
        rospy.sleep(0.8)
        print("SEGMENTANDO....")
        res = segmentation_server.call()
        im=bridge.imgmsg_to_cv2(res.im_out.image_msgs[0])

        #cv2.imshow("Segmentacion",im)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        
        centroids = res.poses.data
        closest = [100,100,100]
        min_dist = 100
        i = 0
        cv2.destroyAllWindows()

        while len(centroids) >= 3:
            print(i)
            centroid = centroids[:3]
            tf_man.pub_static_tf(pos=centroid, point_name='target', ref='head_rgbd_sensor_rgb_frame')
            pos,_=tf_man.getTF(target_frame='target', ref_frame='base_link')
            dist = np.linalg.norm(np.array(pos))
            if dist < min_dist:
                min_dist = dist
                closest = centroid
            centroids = centroids[3:]
            i +=1

        if min_dist < 1.5:
            tf_man.pub_static_tf(pos=closest, point_name='target', ref='head_rgbd_sensor_rgb_frame')
            rospy.sleep(0.8)
            tf_man.change_ref_frame_tf(point_name='target', new_frame='map')
            return 'succ'
        else:
            talk('I did not find any object, I will try again')
            rospy.sleep(0.8)
            return 'failed'
        
        

#---------------------------------------------------
class Gaze_to_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):


        rospy.loginfo('State : GAZE_OBJECT')
        res = omni_base.move_d_to(0.7,'object_gaze')
        rospy.sleep(0.8)

        print('Gazing at : point_Obj')   #X Y YAW AND TIMEOUT
        #hcp = head.absolute(goal_pose[0], goal_pose[1], 0.0)
        head.to_tf('object_gaze')
        rospy.sleep(1.0)
        
        return 'succ'

#---------------------------------------------------
class Grasp_from_floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        #turn to obj goal
        self.tries += 1
        if self.tries == 1:
            head.set_named_target('neutral')
            head.turn_base_gaze('target')
            head.set_named_target('down')
        #pre grasp pose
        brazo.set_named_target('grasp_floor')
        gripper.open()
        rospy.sleep(0.8)
        #get close to object
        brazo.move_hand_to_target(target_frame='target', THRESHOLD=0.01 )
        #move down hand 
        acp = brazo.get_joint_values()
        acp[0]-= 0.04
        brazo.set_joint_values(acp)
        rospy.sleep(0.5)
        #grasp
        gripper.close()
        #move up to check if grasped
        #acp = brazo.get_joint_values()
        acp[0] += 0.09
        brazo.set_joint_values(acp)
        if brazo.check_grasp(weight = 1.0):
            brazo.set_named_target('neutral')
            talk('I took the object')
            rospy.sleep(0.8)
            return 'succ'
        else:
            talk('I could not take the object, i will try again')
            return 'failed'


#---------------------------------------------------
######################################################################################################
class Find_legs(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed', 'tries'])
        self.tries = 0

    def execute(self, userdata):
        self.tries+=1
        if self.tries==6:
            self.tries=0
            return 'tries'
        
        #ENABLE LEG FINDER AND HUMAN FOLLOWER
        msg_bool=Bool()
        msg_bool.data= True
        enable_legs.publish(msg_bool)
        enable_follow.publish(msg_bool)
        ############################
        talk('Leg finder activated')
        print ('Leg finder activated ')
        
        timeout=2
        if self.tries==1:timeout=0.1####FIRST ENABLE TAKES A WHILE
        
        try :
            punto=rospy.wait_for_message("/hri/leg_finder/leg_pose", PointStamped , timeout=timeout)
            print (punto)
            self.tries=0
            return 'succ'
            
            

        except Exception:
            
            print ('No legs found')
            talk( 'I can not  find you, please stand in front of me')
      
            
            msg_bool.data= False
            enable_legs.publish(msg_bool)
            enable_follow.publish(msg_bool)

            return 'failed'    

#########################################################################################################
class Follow_human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'arrived', 'lost'])
        self.tries = 0
        self.last_legs=[]

    def execute(self, userdata):

        rospy.loginfo('STATE : Legs_found,following')
        self.tries+=1
        if self.tries == 1: 
            print('Legs Found, Following')
            talk('Human found, Following')

        try :
            punto=rospy.wait_for_message("/hri/leg_finder/leg_pose", PointStamped , timeout=2.0)
            
        except Exception:
            
            print ('legs _lost')
            talk( 'I lost you, please stand in front of me')
            return 'lost'

        x,y=punto.point.x,    punto.point.y
        
        self.last_legs.append((x,y))
        print(np.linalg.norm(np.asarray(self.last_legs).mean(axis=0)))
        if len (self.last_legs)>=6:
            #if (np.var(self.last_legs,axis=0).mean() < 0.001):
            if (np.linalg.norm(np.asarray(self.last_legs).mean(axis=0)) < 1.0):
                print ('legs stopped... Are we there yet?')#,   np.var(self.last_legs,axis=0).mean()   )     

                talk ('are we there yet ?')
                msg_bool=Bool()
                msg_bool.data= False
                enable_legs.publish(msg_bool)
                enable_follow.publish(msg_bool)
                rospy.sleep(2)
                return 'arrived'
            self.last_legs.pop(0)
        

        
            
        print ('legs moving... Cruising',   np.var(self.last_legs,axis=0).mean()   )          #if (np.var(last_legs,axis=0).mean() < 0.0001):

        return 'succ'

#########################################################################################################


#========================================================================
def init(node_name):
    print('smach ready')
    global reqAct,recognize_action
    global enable_legs,enable_follow

    enable_legs=  rospy.Publisher('/hri/leg_finder/enable', Bool, queue_size=1)
    enable_follow=rospy.Publisher('/hri/human_following/start_follow', Bool, queue_size=1) 


    #rospy.wait_for_service('recognize_act')    
    #rospy.wait_for_service('recognize_face')

    reqAct = RecognizeRequest()
    #train_new_face = rospy.ServiceProxy('new_face', RecognizeFace)    
    #base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    #takeshi_talk_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    
    #sm.userdata.clear = False
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_CARRY_MY_LUGAGGE')
    sis.start()


    with sm:
        #State machine for Restaurant
        smach.StateMachine.add("INITIAL",           Initial(),          transitions = {'failed':'INITIAL',          'succ':'WAIT_PUSH_HAND',    'tries':'END'}) 
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions = {'failed':'WAIT_PUSH_HAND',   'succ':'DETECT_POINT',         'tries':'END'}) 
        smach.StateMachine.add("SCAN_FACE",         Scan_face(),        transitions = {'failed':'SCAN_FACE',        'succ':'DETECT_POINT',      'tries':'INITIAL'}) 
        smach.StateMachine.add("DETECT_POINT",      Detect_action(),    transitions = {'failed':'DETECT_POINT',     'succ':'GAZE_OBJECT',       'tries':'INITIAL'}) 
        smach.StateMachine.add("GAZE_OBJECT",       Gaze_to_object(),   transitions = {'failed':'GAZE_OBJECT',      'succ':'SEGMENT_OBJECT',    'tries':'INITIAL'}) 
        smach.StateMachine.add("SEGMENT_OBJECT",    Segment_object(),   transitions = {'failed':'SEGMENT_OBJECT',   'succ':'GRASP_FROM_FLOOR',  'tries':'INITIAL'}) 
        smach.StateMachine.add("GRASP_FROM_FLOOR",  Grasp_from_floor(), transitions = {'failed':'GRASP_FROM_FLOOR', 'succ':'FIND_LEGS',         'tries':'GRASP_FROM_FLOOR'}) 
        smach.StateMachine.add("FIND_LEGS",          Find_legs(),           transitions={'failed': 'FIND_LEGS',    'succ': 'FOLLOW_HUMAN'    , 'tries': 'END'})
        smach.StateMachine.add("FOLLOW_HUMAN",         Follow_human(),          transitions={'arrived': 'END',    'succ': 'FOLLOW_HUMAN'   , 'lost': 'FIND_LEGS'})
        
    outcome = sm.execute()          