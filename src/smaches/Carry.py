#!/usr/bin/env python3
from utils_takeshi import *

from smach_utils_carry import *



def draw_skeleton(joints,hh,wh,im,cnt_person=0,norm=False,bkground=False,centroid=False):

    """
    conections 15 (of 25) joints:
        0-1   <-> centro cabeza         - cuello-torso
        0-11  <-> centro cabeza         - ojo D ?
        0-12  <-> centro cabeza         - ojo I ?
        1-2   <-> cuello-torso          - hombro D
        1-5   <-> cuello-torso          - hombro I
        1-8   <-> cuello-torso          - tronco-cadera ombligo
        2-3   <-> hombro D              - codo D
        3-4   <-> codo D                - muñeca D
        5-6   <-> hombro I              - codo I
        6-7   <-> codo I                - muñeca I
        8-9   <-> tronco-cadera ombligo - tronco-cadera D 
        8-10  <-> tronco-cadera ombligo -tronco-cadera I 
        11-13 <-> ojo D ?               - oreja D
        12-14 <-> ojo I ?               - oreja I

    conections 14 (of 18) joints:

        0-1  <-> centro cabeza  - cuello-torso
        0-10 <-> centro cabeza  - ojo D
        0-11 <-> centro cabeza  - ojo I
        1-2  <-> cuello-torso   - hombro D
        1-5  <-> cuello-torso   - hombro I
        1-8  <-> cuello-torso   - tronco-cadera D
        1-9  <-> cuello-torso   - tronco-cadera I
        2-3  <-> hombro D       - codo D
        3-4  <-> codo D         - muneca D
        5-6  <-> hombro I       - codo I
        6-7  <-> codo I         - muneca I
        10-12<-> ojo D          - oreja D
        11-13<-> ojo I          - oreja I
 

    """
    h=1
    w=1
    lineThick=2
    circleSize=3

    if norm:
        h=hh
        w=wh

    if bkground:
        bkgn=im.astype(np.uint8)
    else:
        bkgn=np.zeros((hh,wh,3),np.uint8)
    
    if centroid:
        lnCnt=int(joints.shape[0]/2)
        frame=np.zeros((lnCnt,2))
        frame[:,0]=joints[:lnCnt]
        frame[:,1]=joints[lnCnt:]
        if frame.shape[0]==15:
            conections=[[0,1],[0,11],[0,12],[1,2],[1,5],[1,8],
                [2,3],[3,4],[5,6],[6,7],[8,9],[8,10],
                [11,13],[12,14]]
        else:
            conections=[[0,1],[0,14],[0,15],[1,2],[1,5],[1,8],
                [1,11],[2,3],[3,4],[5,6],[6,7],[8,9],
                [9,10],[11,12],[12,13],[14,16],[15,17]]

        
        for conect in conections:
            if frame[conect[0]][0]!=0 and frame[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(frame[conect[0]][0]*h),int(frame[conect[0]][1]*w)),(int(frame[conect[1]][0]*h),int(frame[conect[1]][1]*w)),(0,255,255),lineThick)
        for i in range(frame.shape[0]):
                    if frame[i][0]!=0.0 and frame[i][1]!=0.0:
                        bkgn=cv2.circle(bkgn,(int(frame[i][0]*h),int(frame[i][1]*w)),circleSize,(190,152,253),-1)

        return bkgn

    else:

        if joints.shape[0]==15:
            conections=[[0,1,0],[0,11,1],[0,12,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,10,11],
                        [11,13,12],[12,14,13]]
            # 0-1|0-11|0-12|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-10 
            # 11-13|12-14
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),   
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),   
                    (248,8,207),(248,8,76)]            

        elif joints.shape[0]==18:
            conections=[[0,1,0],[0,14,1],[0,15,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,11,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],
                        [9,10,11],[11,12,12],[12,13,13],[14,16,1],[15,17,1]]
            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        elif joints.shape[0]==25:
            conections=[[0,1,0],[0,15,1],[0,16,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,12,11],
                        [9,10,12],[10,11,13],[11,22,13],[11,24,13],[12,13,14],[13,14,15],
                        [14,19,15],[14,21,15],[15,17,16],[16,18,17],[19,20,15],[22,23,13]]
            # 0-1|0-15|0-16|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-12 
            # 9-10|<10-11|11-22|11-24|22-23>|12-13|<13-14|14-19|14-21|19-20>|15-17|16-18
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),
                    (148,241,4),(239,255,1),(253,145,31),(253,80,31),(248,8,207),(248,8,76)]

        else:  #18 to less joints
            conections=[[0,1,0],[0,10,1],[0,11,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,9,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],
                        [10,12,1],[11,13,1]]

            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        for i in range(joints.shape[0]):
            if joints[i][0]!=0.0 and joints[i][1]!=0.0:
                bkgn=cv2.circle(bkgn,(int(joints[i][0]*h),int(joints[i][1]*w)),circleSize,(255,255,255),-1)

        for conect in conections:
            if joints[conect[0]][0]!=0 and joints[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(joints[conect[0]][0]*h),int(joints[conect[0]][1]*w)),(int(joints[conect[1]][0]*h),int(joints[conect[1]][1]*w)),colors[conect[2]],lineThick)
        
        draw_text_bkgn(bkgn,text="Person:"+str(cnt_person),pos=(int(joints[0,0]), int(joints[0,1])-40),
                   font_scale=1.3,text_color=(255, 255, 32))
        return bkgn
#---------------------------------------------------

def get_extrapolation(mano,codo,z=0):

    vectD=[mano[0]-codo[0],mano[1]-codo[1],mano[2]-codo[2]]
    alfa=z-mano[2]/vectD[2]
    y=mano[1]+alfa*vectD[1]
    x=mano[0]+alfa*vectD[0]
    
    return [x,y,z]
#---------------------------------------------------        
def detect_pointing_arm(lastSK,cld_points):
    area=10
    # CodoD -> 3, CodoI -> 6 
    codoD = [cld_points['x'][round(lastSK[3,1]), round(lastSK[3,0])],
            cld_points['y'][round(lastSK[3,1]), round(lastSK[3,0])],
            cld_points['z'][round(lastSK[3,1]), round(lastSK[3,0])]]
    codoD[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[3,1])-area:round(lastSK[3,1])+area+1, 
                                                round(lastSK[3,0])-area:round(lastSK[3,0])+area+1]))
   
    
    # ManoD -> 4, ManoI -> 7
    manoD = [cld_points['x'][round(lastSK[4,1]), round(lastSK[4,0])],
            cld_points['y'][round(lastSK[4,1]), round(lastSK[4,0])],
            cld_points['z'][round(lastSK[4,1]), round(lastSK[4,0])]]
    manoD[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[4,1])-area:round(lastSK[4,1])+area+1, 
                                                round(lastSK[4,0])-area:round(lastSK[4,0])+area+1]))
    
  
    codoI = [cld_points['x'][round(lastSK[6,1]), round(lastSK[6,0])],
            cld_points['y'][round(lastSK[6,1]), round(lastSK[6,0])],
            cld_points['z'][round(lastSK[6,1]), round(lastSK[6,0])]]
    codoI[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[6,1])-area:round(lastSK[6,1])+area+1, 
                                                round(lastSK[6,0])-area:round(lastSK[6,0])+area+1]))
    
   
    manoI = [cld_points['x'][round(lastSK[7,1]), round(lastSK[7,0])],
            cld_points['y'][round(lastSK[7,1]), round(lastSK[7,0])],
            cld_points['z'][round(lastSK[7,1]), round(lastSK[7,0])]]
    manoI[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[7,1])-area:round(lastSK[7,1])+area+1, 
                                                round(lastSK[7,0])-area:round(lastSK[7,0])+area+1]))
   
    
    tf_man.pub_tf(pos=codoD,point_name='codoD_t',ref='head_rgbd_sensor_link')
    tf_man.pub_tf(pos=codoI,point_name='codoI_t',ref='head_rgbd_sensor_link')
    tf_man.pub_tf(pos=manoD,point_name='manoD_t',ref='head_rgbd_sensor_link')
    tf_man.pub_tf(pos=manoI,point_name='manoI_t',ref='head_rgbd_sensor_link')

    # resta entre [0,-1,0] y vectores de codo a mano 
    
    v1=[-(manoD[0]-codoD[0]),-1-(manoD[1]-codoD[1]),-(manoD[2]-codoD[2])]
    v2=[-(manoI[0]-codoI[0]),-1-(manoI[1]-codoI[1]),-(manoI[2]-codoI[2])]
    
    if np.linalg.norm(v1)<np.linalg.norm(v2):
        print("Mano izquierda levantada")
        return manoI,codoI
    else:
        print("Mano derecha levantada")
        return manoD,codoD

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
        
        '''if self.tries==1:
            head.set_named_target('neutral')

        elif self.tries==2:
            hv= head.get_joint_values()
            hv[1]= hv[1]+0.2 
            head.set_joint_values(hv)

        elif self.tries>=9:
            self.tries=0
            return'tries'''
        head.set_named_target('neutral')        
        rospy.sleep(1.0)
        talk('Please start pointing at the object ')
        print("Detecting action...")
        
        # reqAct.visual --> 1 para visualizar imagen con openpose
        reqAct.visual=0
        # reqAct.in_ --> 3  para detectar pointing sin usar hmm, puro uso de vectores
        reqAct.in_=3
        resAct=recognize_action(reqAct)
        print("Response:",resAct.i_out)
        #sk_xyz=bridge.imgmsg_to_cv2(resAct.sk.image_msgs[0])
        #print(sk)
        #img=draw_skeleton(sk,h,w,img,cnt_person=0,bkground=True)

        #mano,codo=detect_pointing_arm(sk_xyz)
        #tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
        #tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')
        #print("cambiando referencia")

        #tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
        #tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')

        #print("tf publicada y referenciada a map")
        #rospy.sleep(0.8)
        #ob_xyz = get_extrapolation(mano,codo)
        rospy.sleep(0.8)
        posM,_ = tf_man.getTF(target_frame='MANO',ref_frame='map')
        posC,_ = tf_man.getTF(target_frame='CODO',ref_frame='map')
        #print(posM)
        #print(posC)
        extraP=get_extrapolation(posM,posC,z=0)
        #print(extraP)
        tf_man.pub_static_tf(point_name='point_Obj', ref='map', pos= extraP)
        #tf_man.change_ref_frame_tf(point_name='point_Obj', new_frame='map')
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
            talk('I did not find any luggage, I will try again')
            return 'failed'
        
        


class Gaze_to_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):


        rospy.loginfo('State : GAZE_OBJECT')
        #goal_pose, quat=listener.lookupTransform( 'map','Face', rospy.Time(0))
        #obj_pose,_ = tf_man.getTF(target_frame='point_Obj',ref_frame='map')
        #robot_pose,quat_r = tf_man.getTF(target_frame='base_link')
        #yaw=tf.transformations.euler_from_quaternion(quat_r)[2]

        #obj_pose=np.asarray(obj_pose)
        #robot_pose=np.asarray(robot_pose)
        #face_rob = obj_pose-robot_pose
        #goal_pose= obj_pose-(face_rob*0.7/np.linalg.norm(face_rob))

        print('Gazing at : point_Obj')   #X Y YAW AND TIMEOUT
        #hcp = head.absolute(goal_pose[0], goal_pose[1], 0.0)
        head.to_tf('point_Obj')
        rospy.sleep(1.0)
        #move_base(goal_pose[0],goal_pose[1], 0.0 ,20  )   #X Y YAW AND TIMEOUT
        #head.set_joint_value_target(hcp)
        #head.go()
        #tf_man.pub_static_tf(pos=goal_pose, point_name='Goal_D', ref='map')
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
            talk('I took the luggage')
            return 'succ'
        else:
            talk('I could not take the luggage, i will try again')
            return 'failed'


def init(node_name):
    print('smach ready')
    global reqAct,recognize_action


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
        smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions = {'failed':'WAIT_PUSH_HAND',   'succ':'DETECT_POINT',      'tries':'END'}) 
        smach.StateMachine.add("SCAN_FACE",         Scan_face(),        transitions = {'failed':'SCAN_FACE',        'succ':'DETECT_POINT',      'tries':'INITIAL'}) 
        smach.StateMachine.add("DETECT_POINT",      Detect_action(),    transitions = {'failed':'DETECT_POINT',     'succ':'GAZE_OBJECT',       'tries':'INITIAL'}) 
        smach.StateMachine.add("GAZE_OBJECT",       Gaze_to_object(),   transitions = {'failed':'GAZE_OBJECT',      'succ':'END',    'tries':'INITIAL'}) 
        smach.StateMachine.add("SEGMENT_OBJECT",    Segment_object(),   transitions = {'failed':'DETECT_POINT',     'succ':'GRASP_FROM_FLOOR',  'tries':'INITIAL'}) 
        smach.StateMachine.add("GRASP_FROM_FLOOR",  Grasp_from_floor(), transitions = {'failed':'GRASP_FROM_FLOOR', 'succ':'END',               'tries':'GRASP_FROM_FLOOR'}) 
    outcome = sm.execute()          