#!/usr/bin/env python3

from smach_utils2 import * 



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
        
        #arm.set_named_target('go')
        #succ=arm.go()

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
        smach.State.__init__(self,outcomes=['succ','unknown','failed','tries'])
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
        
        print('Checking for faces')
        if res== None:
            return 'failed'
        if res != None:
            print('RESPONSE',res.Ids.ids[0]      )
            if res.Ids.ids[0].data == 'NO_FACE':
                print ('No face Found Keep scanning')
                return 'failed'
            else:
                print ('A face was found.')
                if (res.Ids.ids[0].data=='unknown'):
                    talk('I believe I do not know you')
                    
                    return 'unknown'



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

class New_face(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0

        
    def execute(self,userdata):

        
        rospy.loginfo('STATE : NEW_FACE')
        

        self.tries+=1
        if self.tries==3:
            return 'tries'
                
        head.set_named_target('neutral')
        succ=head.go() 
        talk('Please, tell me your name')
        res=speech_recog_server()
        img=rgbd.get_image()
        res2=train_face(img,res.data)
        print(res2)
        


        

        #arm.set_named_target('go')
        #succ=arm.go()

        #succ = True        
        if succ:
            return 'succ'
        else:
            return 'failed'


def init(node_name):
    print ('smach ready')

    


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
        
        smach.StateMachine.add("INITIAL",           Initial(),          transitions = {'failed':'INITIAL',          'succ':'SCAN_FACE'      ,           'tries':'END'}) 
        smach.StateMachine.add("SCAN_FACE",   Scan_face(),  transitions = {'failed':'SCAN_FACE'          ,  'unknown':'NEW_FACE' ,       'succ':'END'   ,           'tries':'INITIAL'}) 
        smach.StateMachine.add("NEW_FACE",           New_face(),          transitions = {'failed':'INITIAL',          'succ':'END'      ,           'tries':'END'}) 

        
    outcome = sm.execute()          