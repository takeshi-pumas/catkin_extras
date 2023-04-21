#!/usr/bin/env python3
from utils_takeshi import *

from smach_utils_carry import *
def get_extrapolation(mano,codo,z=0):

    vectD=[mano[0]-codo[0],mano[1]-codo[1],mano[2]-codo[2]]
    alfa=z-mano[2]/vectD[2]
    y=mano[1]+alfa*vectD[1]
    x=mano[0]+alfa*vectD[0]
    
    return [x,y,z]


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
        brazo.set_named_target('go')
        #print('arm listo')
        rospy.sleep(0.8)
        return 'succ'

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
        talk('Gently, ...  push my hand to begin')
        succ= wait_for_push_hand(100)

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

class Detect_action(smach.State):
    
    
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
        #class_names=["wave_R","wave_L","neutral","drink","pointing"]
    def execute(self,userdata):

        rospy.loginfo('State : Detect_action')
        self.tries+=1        
        
        if self.tries==1:
            head.set_named_target('neutral')

        elif self.tries==2:
            hv= head.get_joint_values()
            hv[1]= hv[1]+0.2 
            head.set_joint_values(hv)

        elif self.tries>=9:
            self.tries=0
            return'tries'
        rospy.sleep(1.0)
        talk('Please start pointing at the object ')
        print("Detecting action...")
        
        # reqAct.visual --> 1 para visualizar imagen con openpose
        reqAct.visual=0
        # reqAct.in_ --> 3  para detectar pointing sin usar hmm, puro uso de vectores
        reqAct.in_=3
        resAct=recognize_action(reqAct)
        print("Response:",resAct.i_out)

  
        posM,_ = tf_man.getTF(target_frame='MANO',ref_frame='map')
        posC,_ = tf_man.getTF(target_frame='CODO',ref_frame='map')
        #print(posM)
        #print(posC)
        extraP=get_extrapolation(posM,posC,z=0)
        #print(extraP)
        tf_man.pub_static_tf(point_name='point_Obj', ref='map', pos= extraP)
        return 'succ'
        

class Segment_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0

        
    def execute(self,userdata):
        self.tries+=1
        if self.tries==4:return 'tries'

        rospy.loginfo('STATE : Segment_object')
        hv= head.get_joint_values()
 
        #rospy.sleep(1.0)
        print("SEGMENTANDO....")
        res = segmentation_server.call()
        '''print (res.poses.data)
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

            return 'succ'''
        centroids = res.poses.data
        closest = [100,100,100]
        min_dist = 100
        i = 0
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
            talk('I did not find any luggage, I will try again')
            return 'failed'
        
        


class Gaze_to_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):


        rospy.loginfo('State : GAZE_OBJECT')
        tf_man.change_ref_frame_tf(point_name='point_Obj', new_frame='map')
        #goal_pose, quat=listener.lookupTransform( 'map','Face', rospy.Time(0))
        obj_pose,_ = tf_man.getTF(target_frame='point_Obj',ref_frame='map')
        robot_pose,quat_r = tf_man.getTF(target_frame='base_link')
        yaw=tf.transformations.euler_from_quaternion(quat_r)[2]

        obj_pose=np.asarray(obj_pose)
        robot_pose=np.asarray(robot_pose)
        face_rob = obj_pose-robot_pose
        goal_pose= obj_pose-(face_rob*0.7/np.linalg.norm(face_rob))

        print('Gazing at :',goal_pose[0], goal_pose[1], goal_pose[2])   #X Y YAW AND TIMEOUT
        rospy.sleep(1.0)
        hcp = head.absolute(goal_pose[0], goal_pose[1], 0.0)
        #move_base(goal_pose[0],goal_pose[1], 0.0 ,20  )   #X Y YAW AND TIMEOUT
        #head.set_joint_value_target(hcp)
        #head.go()
        tf_man.pub_static_tf(pos=goal_pose, point_name='Goal_D', ref='map')
        return 'succ'

class Grasp_from_floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        #turn to obj goal
        self.tries += 1
        if self.tries == 1:
            head.set_named_target('neutral')
            head.turn_base_gaze('Goal_D')
            head.set_named_target('down')
        #pre grasp pose
        brazo.set_named_target('grasp_floor')
        gripper.open()
        rospy.sleep(0.8)
        #get close to object
        brazo.move_hand_to_target(target_frame='Goal_D', THRESHOLD=0.01 )
        #move down hand 
        acp = brazo.get_joint_values()
        acp[0]-= 0.04
        #grasp
        gripper.close()
        #move up to check if grasped
        #acp = brazo.get_joint_values()
        acp[0] += 0.07
        brazo.set_joint_values(acp)
        if brazo.check_grasp(weight = 1.0):
            brazo.set_named_target('neutral')
            talk('I took the luggage')
            return 'succ'
        else:
            talk('I could not take the luggage, i will try again')
            return 'failed'


def init(node_name):
    print('smach ready')
    #global reqAct,recognize_action


    #rospy.wait_for_service('recognize_act')    
    #rospy.wait_for_service('recognize_face')

    #reqAct = RecognizeRequest()
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
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions = {'failed':'WAIT_PUSH_HAND',   'succ':'DETECT_POINT',      'tries':'END'}) 
        smach.StateMachine.add("SCAN_FACE",         Scan_face(),        transitions = {'failed':'SCAN_FACE',        'succ':'DETECT_POINT',      'tries':'INITIAL'}) 
        smach.StateMachine.add("DETECT_POINT",      Detect_action(),    transitions = {'failed':'DETECT_POINT',     'succ':'GAZE_OBJECT',       'tries':'INITIAL'}) 
        smach.StateMachine.add("GAZE_OBJECT",       Gaze_to_object(),   transitions = {'failed':'GAZE_OBJECT',      'succ':'SEGMENT_OBJECT',    'tries':'INITIAL'}) 
        smach.StateMachine.add("SEGMENT_OBJECT",    Segment_object(),   transitions = {'failed':'DETECT_POINT',     'succ':'GRASP_FROM_FLOOR',  'tries':'INITIAL'}) 
        smach.StateMachine.add("GRASP_FROM_FLOOR",  Grasp_from_floor(), transitions = {'failed':'GRASP_FROM_FLOOR', 'succ':'END',               'tries':'GRASP_FROM_FLOOR'}) 
    outcome = sm.execute()          