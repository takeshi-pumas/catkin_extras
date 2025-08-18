#!/usr/bin/env python3
# Author: Oscar
# ofc1227@tec.mx
import re
from smach_utils2 import *
from restaurant_utils import *
from smach_ros import SimpleActionState
from action_server.msg import GraspAction ,PickPlaceAction, PickPlaceGoal
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
class Wait_for_waving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0
        self.gaze = [[ 0.0, 0.05],[ 0.0, 0.05],[ -0.5, 0.15],[ 0.5, 0.15],[ 0.85, 0.2],[ -0.85, 0.2]]
    def execute(self, userdata):
        #req = RecognizeRequest()    # DOCKER
        req = RecognizeOPRequest() # NORMAL

        # Guardo posicion para retornar
        trans, quat=tf_man.getTF('base_link')
        tf_man.pub_static_tf(pos=trans, rot =[0,0,0,1], point_name='INITIAL_PLACE')
        rospy.loginfo('STATE : Wait for a person waving') 
        if self.tries < 5:self.tries += 1
        else: 
            self.tries=0
            return 'tries'
        
        print(f'[WAITFORWAVING]Try {self.tries} of N attempts')
        
        talk('Looking for a person waving')
        print('[WAITFORWAVING] Looking for a waving person')
        print(f"[WAITFORWAVING] gaze:{self.gaze[self.tries]}")
        head.set_joint_values(self.gaze[self.tries])

        rospy.sleep(1)
        req.visual=0
        # reqAct.in_ --> 5  para detectar Waving en Restaurant
        req.in_ = 5
        talk("Three")
        print("[WAITFORWAVING] 3")
        #rospy.sleep(1.0)
        talk('Two')
        print("[WAITFORWAVING] 2")
        #rospy.sleep(1.0)
        talk('One')
        print("[WAITFORWAVING] 1")
        #rospy.sleep(1.0)

        #resAct=recognize_action_docker(req) # DOCKER
        resAct=recognize_action(req)       # NORMAL
        
        print("[WAITFORWAVING] RES OF SERVICE:",resAct.i_out)


        #img = bridge.imgmsg_to_cv2(resAct.im_out.image_msgs[0])
        save_image(bridge.imgmsg_to_cv2(resAct.im_out.image_msgs[0]),name="wavingRestaurant")
        if resAct.i_out == 1:
            talk('I found someone waving')
            print("[WAITFORWAVING] I found someone waving")
        else:
            talk('I did not found someone waving')
            print("[WAITFORWAVING] I did not found someone waving")
        return 'failed' if self.tries < 6 else 'tries'
        
        

        talk('Aproaching to the waving person')
        print('[WAITFORWAVING] Aproaching to the waving person')
        self.tries=0
        return 'succ'
#--------------------------------------------------
class Goto_waving_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed','tries'])
        self.tries = 0

    def execute(self, userdata):
        rospy.loginfo('STATE : Navigate to forbidden room')

        print(f'[GOTOWAVINGPERSON] Try {self.tries} of 3 attempts')
        self.tries += 1
        if self.tries == 3:
            return 'tries'
        
        human_xyz,_=tf_man.getTF('person_waving',ref_frame='base_link')
        print(f'[GOTOWAVINGPERSON] coords {human_xyz}')

        res = omni_base.move_d_to(0.7,'person_waving')
        # publica pointStamp para que se mueva con potFields o lo que se vaya a usar
        #res = new_move_D_to(tf_name='person_waving',d_x=1 , timeout=20.0)
        print("[GOTOWAVINGPERSON]",res)

        if res:
            # move_base() # with no map
            self.tries = 0
            talk('Arrrived to person')
            rospy.sleep(1)
            return 'succ'
        else:
            print('[GOTOWAVINGPERSON] Navigation Failed, retrying')
            return 'failed'


####################################################################################################
class Plan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'navigate','pick','place'
                                            ,'find_objects','follow' ,'answer'
                                            ,'locate_person', 'tell_joke'],
                                            output_keys=['actions','params','command_id'],input_keys=['actions','params'])
        self.tries = 0

    def execute(self, userdata):

        rospy.loginfo('STATE : Execute PLAN')
        actions=userdata.actions
        talk (actions[0]) 
        print(f'Ollama plan to actions{actions} ')
        if len(actions)==0:
            print("I believe I have suceeded. Go back to initial and wait for command" )
            return 0
        else:
            current_action=camel_to_snake(actions[0]).strip().lower()   ### LLMS Still do wierd things sometims
            
            if   current_action == 'navigate':return'navigate'
            elif current_action =='pick_object':return'pick'
            elif current_action =='place_object':return'place'
            elif current_action =='find_object':return'find_objects'
            elif current_action =='follow_person':return'follow'
            elif current_action =='answer_question':return'answer'
            elif current_action =='tell_joke':return 'tell_joke'
            # elif current_action =='recognize_action':return 'person_action'
            # elif current_action =='recognize_posture':return 'person_posture'
            elif current_action in ['locate_person','identify_person','greet_person']:return 'locate_person'
            elif current_action =='tell_joke':return 'tell_joke'
            


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
        talk('Ready for GPSR, waiting for comand using QR please place it where I can see it.')   
        print('Ready for GPSR, waiting for comand using QR')   
        
        #plan="plan=['FindObject(EndTable)', 'Navigate(EndTable)', 'IdentifyObject(largest_object)', 'ReportObjectAttribute(size)']" #Example Desired PLAN
        #plan="[ navigate('dinning_table') , FindObject('potted_meat_can'), PickObject('potted_meat_can'), navigate('start_location')]"        
        req = ActionPlannerRequest()
        req.timeout= 50
        command=action_planner_server(req)
        print(f'command.plan.data{command.plan.data}.')
        plan=command.plan.data
        if command.plan.data== 'timed out':return 'failed'
        ########################
        #plan=command.plan.data
        print(plan,'plan \n')

        # Use regex to match function calls like FunctionName(arg1, arg2, ...)
        pattern = re.compile(r'(\w+)\((.*?)\)', re.DOTALL)

        actions = []
        params = []

        for match in pattern.finditer(plan):
            action_name = match.group(1).strip()
            arguments = match.group(2).strip().strip('\'"').strip()
            actions.append(action_name)
            if action_name == "Navigate":arguments = arguments.replace(" ", "_")
            params.append(arguments)

        # Output result
        for a, p in zip(actions, params):
            print(f"Action: {a}, Params: {p}")


        print (actions, params)
        userdata.actions=actions
        userdata.params=params
        if len (actions)>0:return 'succ'
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
        
#########################################################################################################
class Get_command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed', 'tries'],output_keys=['actions','params'])
        self.tries = 0

    def execute(self, userdata):
            self.tries += 1
            rospy.loginfo('STATE : GET DRINK')
            if self.tries == 4:
                talk ('I am having trouble understanding you, lets keep going')
                self.tries=0
                return 'tries'
            
            talk('Do you have a command for me?')

            #rospy.sleep(2.5)
            confirmation = get_keywords_speech(10)
            confirmation = confirmation.split(' ')
            confirm = match_speech(confirmation, ['yes','yeah','jack','juice'])
 
            if not confirm: return 'tries' 
            
            talk('PLease, give the command')
            #rospy.sleep(2.0)
            cmd = get_keywords_speech(20)

            if cmd=='timeout':
                talk("Sorry, couldn't hear you. Please speak louder.")
                return 'failed'

            talk("Is this the command? ...")
            talk(cmd)
            
            self.tries = 0
            confirmation = get_keywords_speech(10)
            confirmation = confirmation.split(' ')
            confirm = match_speech(confirmation, ['yes','yeah','jack','juice'])
 
            if not confirm:
                talk("Let's try again") 
                return 'failed' 
            

            #plan="plan=['FindObject(EndTable)', 'Navigate(EndTable)', 'IdentifyObject(largest_object)', 'ReportObjectAttribute(size)']" #Example Desired PLAN
            #plan="[ navigate('dinning_table') , FindObject('potted_meat_can'), PickObject('potted_meat_can'), navigate('start_location')]"        
            req = ActionPlannerRequest()
            req.command.data = cmd
            command=action_planner_server(req)
            print(f'command.plan.data{command.plan.data}.')
            plan=command.plan.data
            if command.plan.data== 'timed out':return 'failed'
            ########################
            #plan=command.plan.data
            print(plan,'plan \n')

            # Use regex to match function calls like FunctionName(arg1, arg2, ...)
            pattern = re.compile(r'(\w+)\((.*?)\)', re.DOTALL)

            actions = []
            params = []

            for match in pattern.finditer(plan):
                action_name = match.group(1).strip()
                arguments = match.group(2).strip().strip('\'"').strip()
                actions.append(action_name)
                if action_name == "Navigate":arguments = arguments.replace(" ", "_")
                params.append(arguments)

            # Output result
            for a, p in zip(actions, params):
                print(f"Action: {a}, Params: {p}")


            print (actions, params)
            userdata.actions=actions
            userdata.params=params
            if len (actions)>0:return 'succ'
            return 'failed'

#########################################################################################################    
class Go_to_next_room(smach.State):  # ADD KNONW LOCATION DOOR
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.tries = 0
        self.next_room=1

    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')

        print(f'Try {self.tries} of 3 attempts')
        self.tries += 1
        
        
        
        if self.tries   <= 1: 
            talk('Navigating to  living room')
            next_room='living_room'
        elif self.tries == 2: 
                    talk('Navigating to dining room')
                    next_room='dining_room'
        elif self.tries == 3: 
                    talk('Navigating to kitchen')
                    next_room='bedroom'
        elif self.tries == 4: 
                    talk('Navigating to bedroom')
                    next_room='kitchen'
                    self.tries=0
       

        res = omni_base.move_base(known_location=next_room)
        print(res)

        if res :


            return 'succ'

        else:
            talk('Navigation Failed, retrying')
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
class Locate_person(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params'])
        
    def execute(self, userdata):

        rospy.loginfo('STATE : find human in current location')
        print(f'Navigating to,{userdata.params[0]}')        
        if self.tries==1:
            talk('Scanning the room for humans')
            rospy.loginfo('Scanning the room for humans')
            head.set_joint_values([ 0.0, 0.1])# Looking ahead
        if self.tries==2:head.set_joint_values([ 0.5, 0.1])#looking left
        if self.tries==3:head.set_joint_values([-0.5, 0.1])#looking right        
        rospy.sleep(1.0)
        res=detect_human_to_tf()
        print (f'Res to detect human{res} , actions{userdata.actions}params{userdata.params}')



        if res:
            userdata.actions.pop(0)
            userdata.params.pop(0)
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'
##########################################################
# class Recognize_action(smach.State):  
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params','result'], input_keys=['actions','params'])
        
#     def execute(self, userdata):

#         rospy.loginfo('STATE : recognize human action in current location')        
#         if self.tries==1:
#             talk('recognizing human action')
#             rospy.loginfo('Scanning the room for humans')
#             head.set_joint_values([ 0.0, 0.1])# Looking ahead
#         if self.tries==2:head.set_joint_values([ 0.5, 0.1])#looking left
#         if self.tries==3:head.set_joint_values([-0.5, 0.1])#looking right        
#         rospy.sleep(1.0)
#         keypoints = get_keypoints()
#         res=recognize_action(keypoints)
#         print (f'Res to recognize human action {res} , actions{userdata.actions}params{userdata.params}')



#         if res:
#             userdata.actions.pop(0)
#             userdata.params.pop(0)
#             userdata.result = res
#             return 'succ'
#         else:
#             talk('recognition Failed, retrying')
#             return 'failed'
# ##########################################################

class Recognize_posture(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params','result'], input_keys=['actions','params'])
        
    def execute(self, userdata):

        rospy.loginfo('STATE : recognize human posture in current location')        
        if self.tries==1:
            talk('recognizing human posture')
            rospy.loginfo('Scanning the room for humans')
            head.set_joint_values([ 0.0, 0.1])# Looking ahead
        if self.tries==2:head.set_joint_values([ 0.5, 0.1])#looking left
        if self.tries==3:head.set_joint_values([-0.5, 0.1])#looking right        
        rospy.sleep(1.0)
        keypoints = get_keypoints()
        res=recognize_posture(keypoints)
        print (f'Res to recognize human action {res} , actions{userdata.actions}params{userdata.params}')



        if res:
            userdata.actions.pop(0)
            userdata.params.pop(0)
            userdata.result = res
            return 'succ'
        else:
            talk('recognition Failed, retrying')
            return 'failed'
##########################################################




class ReportResult(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params','result'])
        
    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        location = userdata.params[0].strip('\'"')
        print(f"Navigating to: {location}")

        #talk('Navigating to, pickup')        
        res = omni_base.move_base(known_location='start_location', time_out=40)
        print(res)

        if res:
            talk (f'the result is {userdata.result}')
            print(f'the result is {userdata.result}')
            userdata.actions.pop(0)
            userdata.params.pop(0)
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'
##########################################################
class Navigate(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params'])
        
    def execute(self, userdata):

        rospy.loginfo('STATE : Navigate to known location')
        location = userdata.params[0].strip('\'"')
        print(f"Navigating to: {location}")

        #talk('Navigating to, pickup')        
        res = omni_base.move_base(known_location=location, time_out=40)
        res = omni_base.move_base(known_location=location, time_out=40)
        print(res)

        if res:
            userdata.actions.pop(0)
            userdata.params.pop(0)
            return 'succ'
        else:
            talk('Navigation Failed, retrying')
            return 'failed'
##########################################################
class Tell_joke(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params'])
        
    def execute(self, userdata):
        rospy.loginfo('STATE : Tell Joke')
        req = ActionPlannerRequest()
        req.command.data="tell Joke?"
        command=action_planner_server(req)
        print(f'command.plan.data{command.plan.data}.')
        talk(command.plan.data)

        if res:
            userdata.actions.pop(0)
            userdata.params.pop(0)
            return 'succ'
        else:
            return 'failed'


class Answer_question(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'failed'], output_keys=['actions','params'], input_keys=['actions','params'])
        
    def execute(self, userdata):
        rospy.loginfo('STATE : Tell Joke')
        req = ActionPlannerRequest()
        req.command.data[0].strip('\'"') + "?"
        command=action_planner_server(req)
        print(f'command.plan.data{command.plan.data}.')
        talk(command.plan.data)

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
###################################################################################
class CountObjects(smach.State):
    
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succ', 'failed'], output_keys=['actions','params','result'], input_keys=['actions','params'])
    
    def execute(self, userdata):
        rospy.loginfo('State : Counting Objects')
        
        if self.tries==0:
            talk(f'Counting {userdata.params[0]}')
            head.set_joint_values([ 0.0, -0.5])
        if self.tries==3:
             talk( 'I could not find more objects, ask for human assistance')
             return 0  #TODO  escape and ask
        
        rospy.sleep(5.0)                        
        img_msg  = bridge.cv2_to_imgmsg( cv2.cvtColor(rgbd.get_image(), cv2.COLOR_RGB2BGR))### GAZEBO BGR!?!??!
        req      = classify_client.request_class()
        req.in_.image_msgs.append(img_msg)        
        res      = classify_client(req)
        objects, poses=detect_object_yolo('all',res)   
        if len (objects)!=0 :
            cats=[]
            for name in objects:cats.append(categorize_objs(name))
            count = cats.count(userdata.params[0])
            print(count) 
            userdata.result=count  
            userdata.actions.pop(0)
            userdata.params.pop(0)         
            return 'succ'
        else:
            print('Objects list empty')            
            self.tries+=1
            return 'failed'
        
        
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
        find_placing_area(plane_height=-1)###AUTO

        pos, quat = tf_man.getTF(target_frame = 'placing_area', ref_frame = 'map')
        rospy.sleep(1.0)                
        offset_point=[0.0,0.0,0.15 ]
        pos[2]=pos[2]+offset_point[2]
        tf_man.pub_static_tf(pos=pos,rot=quat,point_name='goal_for_place')
        rospy.sleep(0.5)
        ##################################################################3

        pos, quat = tf_man.getTF(target_frame = 'goal_for_place', ref_frame = 'map')
        print (f"pos{pos}")
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'map'
        pose_goal.header.stamp = rospy.Time.now()
        pose_goal.pose.position.x=   pos[0]
        pose_goal.pose.position.y=   pos[1]
        pose_goal.pose.position.z=   pos[2]
        pose_goal.pose.orientation.x=quat[0]
        pose_goal.pose.orientation.y=quat[1]
        pose_goal.pose.orientation.z=quat[2]
        pose_goal.pose.orientation.w=quat[3]
        ########################################
        userdata.target_pose = pose_goal
        print (pose_goal)
        ###################
        head.set_named_target('neutral')
        string_msg = String()
        string_msg.data='frontal'                
        userdata.mode=string_msg             
        ###################################
        rospy.sleep(0.5)
        return 'succ'     
###################################
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
    sm.userdata.actions=[]
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()
    with sm:
        # State machine STICKLER
        smach.StateMachine.add("INITIAL"            ,   Initial(),          transitions={'failed': 'INITIAL',           
                                                                                         'succ': 'GO_TO_NEXT_ROOM',   
                                                                                         'tries': 'END'})
        
        smach.StateMachine.add("GO_TO_NEXT_ROOM", Go_to_next_room(),     
                               transitions={'failed': 'GO_TO_NEXT_ROOM','succ': 'WAITING_WAVING'})

        # Interaction
        # smach.StateMachine.add("GO_TO_INSTRUCTION_ROOM", Go_to_instruction_room(),  
        #                        transitions={'failed': 'GO_TO_INSTRUCTION_ROOM', 'succ': 'WAITING_WAVING'})
        
        smach.StateMachine.add("WAITING_WAVING", Wait_for_waving(),            
                               transitions={'failed': 'WAITING_WAVING', 'succ': 'GOTO_WAVING_PERSON', 'tries':'GO_TO_NEXT_ROOM'})
        
        
        smach.StateMachine.add("GOTO_WAVING_PERSON", Goto_waving_person(),    
                               transitions={'failed': 'GOTO_WAVING_PERSON', 'succ': 'GET_COMMAND','tries':'GO_TO_NEXT_ROOM'})
        
        
        smach.StateMachine.add("GET_COMMAND"       ,   Get_command(),       transitions={'failed': 'GET_COMMAND',    
                                                                                         'succ': 'PLAN',       
                                                                                         'tries': 'GO_TO_NEXT_ROOM'})
        smach.StateMachine.add("PLAN"               ,   Plan(),        transitions={'failed': 'PLAN', 
                                                                                        'succ': 'END',
                                                                                        'pick':'PICKUP',
                                                                                        'place':'PLACE',  
                                                                                        'find_objects':'FIND_OBJECTS',  
                                                                                        'navigate': 'NAVIGATE',
                                                                                        'follow'  :'FOLLOW_PERSON',
                                                                                        'answer'  :'ANSWER_QUESTION',
                                                                                        'tell_joke'  :'TELL_JOKE',
                                                                                        'locate_person':'LOCATE_PERSON',
                                                                                        #'recognize_action':'RECOGNIZE_ACTION',
                                                                                        #'recognize_posture':'RECOGNIZE_POSTURE'
                                                                                         })
        smach.StateMachine.add("LOCATE_PERSON"               ,   Locate_person(),       transitions={'failed': 'LOCATE_PERSON',    
                                                                                        'succ': 'PLAN', 
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
        
        smach.StateMachine.add("TELL_JOKE"        ,   Tell_joke(),     transitions={'failed': 'TELL_JOKE',    
                                                                                         'succ': 'PLAN', 
                                                                                         })
        

        
        smach.StateMachine.add("PICKUP",    Pickup(),                   transitions={'failed': 'PICKUP',    
                                                                                         'succ': 'GRASP_GOAL'})
        smach.StateMachine.add("PLACE",    Place(),                     transitions={'failed': 'PLACE',    
                                                                                         'succ': 'PLACE_GOAL',
                                                                                         'tries': 'PLAN'})
        # smach.StateMachine.add("RECOGNIZE_ACTION",    Recognize_action(),                     transitions={'failed': 'RECOGNIZE_ACTION',    
        #                                                                                  'succ': 'PLAN',
        #                                                                                  'tries': 'PLAN'})
        # smach.StateMachine.add("RECOGNIZE_POSTURE",    Recognize_posture(),                     transitions={'failed': 'RECOGNIZE_POSTURE',    
        #                                                                                  'succ': 'PLAN',
        #                                                                                  'tries': 'PLAN'})

        smach.StateMachine.add("GRASP_GOAL", SimpleActionState('grasp_server', GraspAction, goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),                      
                                transitions={'preempted': 'PLAN', 'succeeded': 'CHECK_GRASP', 'aborted': 'PLAN'})
       
        smach.StateMachine.add("PLACE_GOAL", SimpleActionState('place_server', PickPlaceAction, goal_slots={'target_pose': 'target_pose', 'mode': 'mode'}),                    
                                transitions={'preempted': 'PLACE', 'succeeded': 'POST_PLACE', 'aborted': 'PLACE'})
        
        smach.StateMachine.add("ID_PERSON", SimpleActionState('id_server', IdentifyPersonAction, goal_slots={'command_id': 'command_id'}),
                               transitions={'aborted': 'ID_PERSON','succeeded': 'PLAN' ,'preempted': 'PLAN' })
        
        
        smach.StateMachine.add("FOLLOW_PERSON", SimpleActionState('follow_server', FollowAction), transitions={'aborted': 'FOLLOW_PERSON',        
                                                                                                            'succeeded': 'PLAN' ,   
                                                                                                            'preempted': 'PLAN' })
        
        
                        
        # smach.StateMachine.add("WAIT_PUSH_HAND",    Wait_push_hand(),   transitions={'failed': 'WAIT_PUSH_HAND',    
        #                                                                                  'succ': 'WAIT_COMMAND',       
        #                                                                                  'tries': 'END'})

        # smach.StateMachine.add("WAIT_DOOR",    Wait_door_opened(),      transitions={'failed': 'WAIT_DOOR',    
        #                                                                                  'succ': 'WAIT_COMMAND'})

        
        
        smach.StateMachine.add("CHECK_GRASP",    Check_grasp(),         transitions={'failed': 'PLAN',    
                                                                                         'succ': 'PLAN',
                                                                                         })
        ###################################################################################################################################################################
        smach.StateMachine.add("POST_PLACE",    Post_place(),         transitions={'failed': 'PLAN',    
                                                                                         'succ': 'PLAN',
                                                                                         })
        ###################################################################################################################################################################
        


    outcome = sm.execute()
