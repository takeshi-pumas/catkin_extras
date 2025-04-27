#!/usr/bin/env python
# Author: Oscar
# ofc1227@tec.mx
import rospy
import rospkg
import yaml
from action_planner.srv import ActionPlanner, ActionPlannerResponse
from std_msgs.msg import String  # Import String message type
from pyzbar import pyzbar
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
from ollama import chat
import re #regex
################
locations_names="""bed (p) 
bedside table (p) 
shelf (p) | cleaning supplies 
trashbin 
dishwasher (p) 
potted plant 
kitchen table (p) | dishes 
chairs 
pantry (p) | food 
refrigerator (p) 
sink (p) 
cabinet (p) | drinks 
coatrack 
desk (p) | fruits 
armchair 
desk lamp 
waste basket 
tv stand (p) 
storage rack (p) 
lamp 
side tables (p) | snacks 
sofa (p) 
bookshelf (p) | toys 
entrance 
exit 
"""
tools="""
    def Navigate(target_location: str):
        \"\"\"
        Move the robot to a target location.
        \"\"\"
    
    def FollowPerson(person_name: str):
        \"\"\"
        Follow a specific person as they move.
        \"\"\"
    
    def PickObject(object_name: str):
        \"\"\"
        Pick an object.
        \"\"\"
    
    def PlaceObject(object_name: str, target_location: str):
        \"\"\"
        Place an object at a target location.
        \"\"\"
    
    def GreetPerson(person_name: str):
        \"\"\"
        Greet a person verbally or visually.
        \"\"\"
    
    def AnswerQuestion(question: str):
        \"\"\"
        Respond to a question from a person.
        \"\"\"
    
    def IdentifyPerson(person_name: str):
        \"\"\"
        Provide information about a person (pose, clothing, gender, etc.).
        \"\"\"
    
    def TellJoke(person_name: str):
        \"\"\"
        Tell a joke to entertain or engage a person.
        \"\"\"
    
    def TellStatement(info_item: str):
        \"\"\"
        Inform something to someone (robot assumes it already navigated to the person).
        \"\"\"
    
    def StateInfo(info_item: str):
        \"\"\"
        State contextual information (e.g., time, date).
        \"\"\"
    
    def FindObject(object_name: str = None, object_type: str = None):
        \"\"\"
        Locate a specific object in the environment.
        \"\"\"
    
    def CountObjects(type_object: str, location: str = None):
        \"\"\"
        Count the number of objects of a specific type in a location.
        \"\"\"
    
    def ReportObjectAttribute(attribute: str, object_name: str = None):
        \"\"\"
        Identify attributes of an object.
        \"\"\"
    
    def LocatePerson(person_name: str):
        \"\"\"
        Identify the location of a specific person.
        \"\"\"
    """


######################################################################
class RGB:
    def __init__(self):
        # Get the topic name from the parameter server
        #self.image_topic = rospy.get_param('/image_topic', '/usb_cam/image_raw')  # Default to /hsrb/head_rgbd_sensor/rgb/image_rect_color
        self.image_topic = rospy.get_param('/image_topic', "/hsrb/head_r_stereo_camera/image_raw")  # Default to /hsrb/head_rgbd_sensor/rgb/image_rect_color

        self.bridge = CvBridge()

    def get_image(self):
        """
        Waits for an image message from the specified topic and converts it to an OpenCV image.

        :return: OpenCV image
        """
        try:
            rospy.loginfo(f"Waiting for an image on topic: {self.image_topic}")
            image_msg = rospy.wait_for_message(self.image_topic, ImageMsg, timeout=10)  # 10-second timeout
            rospy.loginfo("Image received.")
            return self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to receive image: {e}")
            return None

def wait_for_qr(img,timeout=10):
      # Initialize the RGB class
    decoded_objects = []
    start_time = rospy.get_time()
    
    while rospy.get_time() - start_time < timeout:
        print ("waiting for qr")
        img=rgb.get_image()
        if img is not None:
            decoded_objects = pyzbar.decode(img)
            rospy.loginfo(f"Decoded objects count: {len(decoded_objects)}")
            if len(decoded_objects) != 0:
                return decoded_objects[0].data.decode('utf-8')
        else: print('no imgs')
    return 'time out'

######################################################################################        
def answer_question(question):
    
    print('Answering... wait for a few seconds ')
    messages = [
    {
          "role": "user",
           "content": question
        },
    ]
    response = chat('mistral', messages=messages)
    print(response['message']['content'])
    
    return response['message']['content']
######################################################################################        
def read_yaml(known_locations_file='/known_locations.yaml'):
    rospack = rospkg.RosPack()
    file_path=rospack.get_path('action_planner')+'/context_files/'+ known_locations_file

    with open(file_path, 'r') as file:
        content = yaml.safe_load(file)
    return content

######################################################################################        




import yaml
import rospkg


def plan_command(command):
    #model="llama3.2"
    model="mistral"
    
    
    
    
    #known_locs=read_yaml('known_locs_gpsr.yaml')
    #######################################################################################        
    #rospack = rospkg.RosPack()                                                                                  # THIS CAN BE YAMLD TODO
    #file_path=rospack.get_path('action_planner')+'/context_files/examples.txt'
    #with open(file_path, 'r') as file:
    #    examples = file.read()
    known_locs=read_yaml('known_locs_gpsr.yaml')
    print( 'Planing please wait')

    prompt_1 = (
                f" You are an action planner, a service robot will execute the sequence of actions you create"
                f" Service robot only has this actions {tools} implemented if plan contains any action not inlcuded here it is considered a catastrophic planning failiure"
                f" Assume you can use Navigate() to go to any location mentioned in {locations_names}"
                f" After each location name there is a (p) for locations in which robot can place objects and aslo a category of objects"
                "that migh be found "
                f" Robot always starts at 'start' location."
                f"User command is {command}"
                )
    messages = [{"role": "tool", "content": prompt_1}]
    # First call to the model
    response = chat(model, messages=messages)
    plan_yaml = response['message']['content']
    print (plan_yaml)
    # Second prompt to reformat the plan into the desired action list format
    prompt_2 = (
        f"Based on this plan\n{plan_yaml}\n\n"
        f"Make sure all actions are part of the action set{tools}.\n\n"
        f"Make sure all actions have parameters.\n\n"
        f"Reformat the actions into this format: "
        f"plan=[name_of_action_from_action_set(parameter), name_of_action_from_action_set(parameter), etc]. "
        "Please ensure:"
            "- Each action is a function call in the format: function_name(parameter) from the action set do not use any other actions not included "
            "- Use the exact Python function syntax with no extra characters (e.g., no quotes around function names, no `=` between the function name and parameter)."
            "- All parameters should be inside the parentheses and should not have additional quotes unless they are string arguments."
            f"- The entire output should be a Python list format of actions in {tools}"
            "- avoid any signs like single quotes or  '\n' avoid unnecesary spaces"
    )

    messages = [
        {"role": "tool", "content": prompt_1},
        {"role": "tool", "content": plan_yaml},
        {"role": "tool", "content": prompt_2},
    ]
    response2 = chat(model, messages=messages)
    plan_list = response2['message']['content']
    #print(plan_list) 

    return plan_list
    



######################################################################################        
def fact_check(plan):
    print('Checking Facts ')
    known_locs=read_yaml('known_locs_gpsr.yaml')
    messages = [
        {
            "role": "tool",
            "content": (
                f"Here is the command: {command}. You are a fact checker. "
                f"Fact: You are talking to a user, and you have correct maps and localization from your location and his"
                f"Fact: You know the location of every entity contained in {known_locs} so no question is needed if the location mentioned in command exist here '"
                f"Fact: objects and persons' locations are known only if they are listed in {known_locs}. "
                f"Fact: objects and person locations not included in {known_locs} are assumed to be dynamic. so unless the info is contained in {command} You should ask the user where the object or person is ."                f"Do not add new entities or locations beyond what is listed in {known_locs} or the command. "
                f"Only provide the necessary action steps. "
                f"If there is missing information (e.g., object location), ask a short question to get the missing information from the user. Stating Question: Ask user short quesiton  "
                f"ensure your question starts with ask and ends with a question mark."
                
               
            )
        },
    ]
    #response = chat('mistral', messages=messages)
    response = chat('bespoke-minicheck', messages=messages)
    print(response['message']['content'].lower())
    response_content=response['message']['content']
     
    # Handle the case where missing information is identified
    if "ask" in response_content.lower() or "where" in response_content.lower() or "need" in response_content.lower():
        print("There may be missing information. Please provide more details.")
        question_match = re.search(r"(.*\?)", response_content)  # Find any sentence ending with a question mark
        if question_match:
            extracted_question = question_match.group(0)
            print(f"Question extracted: {extracted_question}")
            return  extracted_question
        else:
            print("No question found.")
            return  "continue"
    else:
            print("No question found.")
            return  "continue"


####################################################3
def next_action(plan):
    robot_state= {
            "robot_location": "initial_location",
            "robot_holding": None

        }


    # Step 2: Reformat the plan into a list format
    prompt = (
        f" We have a plan{plan}, which should be made only from available actions to the service robot that will use your high level plan"
        f" It is very important to keep the plan the same, just substitute actions not found in the following action set"
        f" The actions are {tools}.ONLY ACTIONS from these set are allowed in the thought process."
        f" What action should be called first, "
        f" show example call of how the action chosen must be called according to description, "
        f" how would the current  robot state{robot_state} would be updated if the first action is succesfully performed?"
        f" Answer must only inlcude next_action , updated robot state in exactly "
        f" let me emphasize the importance to the highest degree   that the syntax beeing exacly like this "
        f" next action: example of call, an axample would look like this next action:navigate(living_room)  , updated state: updated robot state "
        f" and only include calls from {tools} "


    )
    messages = [

        {"role": "tool", "content": prompt},

    ]
    response2 = chat(model='mistral', messages=messages)
    plan_list = response2['message']['content']
    print(plan_list)
    return plan_list

######################################################################################        
import re
import ast  # for safely parsing the dict

def parse_llm_step_response(response_text):
    # Flexible match for something like: def Navigate('living room') or Navigate("kitchen")
    action_match = re.search(r'next action:\s*(?:def\s*)?(\w+)\s*\(\s*[\'"](.+?)[\'"]\s*\)', response_text, re.IGNORECASE)
    state_match = re.search(r'updated state:\s*(\{.*\})', response_text)

    next_action = action_match.group(1) if action_match else None
    next_param = action_match.group(2) if action_match else None

    try:
        updated_state_str = state_match.group(1) if state_match else '{}'
        # Less strict version — use eval if you're sure of format, or a simple string parser otherwise
        updated_state = eval(updated_state_str) if updated_state_str else {}
    except Exception as e:
        updated_state = {}
        print("Failed to parse updated state:", e)

    return next_action, next_param, updated_state

######################################################################################        



######################################################################################        




def check_format(plan):
    messages = [
    {
          "role": "tool",
           "content": f" Inforce the syntax of this {plan}, Notice there are actions and parameters in this form  name_of_action(parameters)  all parameters must follow snake_case convention"
        },
    ]


    response = chat('mistral', messages=messages)
    print(response['message']['content'])
    return response['message']['content']
######################################################################################        

def action_planner(req):
    global command
    rospy.loginfo(f"Received: {req.command}")
    response = ActionPlannerResponse()  # Create a new String message
    if len(req.command.data)!=0:
        if req.command.data[-1]=='?':#  What question will be answered?
            print (f'question: {req.command.data}')
            answer=answer_question(req.command.data)
            response.plan.data=answer
        else:
            plan=plan_command(req.command.data)
            print (f'plan{plan}')
            #check_plan=fact_check(plan)
            response.plan.data = plan
    else:
        qr_text=wait_for_qr(req.timeout)
        print(qr_text)
        
        print ('planning',qr_text)
        if qr_text != "time out":
            command=qr_text
            plan=plan_command(command)
            ########################
            print(plan,'plan \n')
            response.plan.data = plan
            start = plan.find("[") + 1
            end = plan.rfind("]")
            inner = plan[start:end]
            ##########################
            nxt_action= next_action(plan)
            print(nxt_action,'action \n')
            action,param,state=parse_llm_step_response(nxt_action)
            print (f'action,param,state{action,param,state}')
            response.next_action.data = action
            response.next_param.data = param



        else:
            print (' Qr Timed OuT')
            response.plan.data ='timed out'
    return response

if __name__ == "__main__":
    global rgb
    rospy.init_node('action_planner_server')
    
    # Load the parameter for the image topic
    rospy.loginfo("Waiting for parameter 'image_topic'...")
    if rospy.has_param('/image_topic'):
        rospy.loginfo(f"Using image topic: {rospy.get_param('/image_topic')}")
    else:
        rospy.logwarn("Parameter 'image_topic' not found. Using default 'right eye takeshi'.")

    service = rospy.Service('action_planner', ActionPlanner, action_planner)
    rgb = RGB()
    rospy.loginfo("Action Planner Service Ready")
    rospy.spin()
