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
locations_names="""
 bedside table (p) |
 side table (p) | snacks |
 bed (p) |
 kitchen table (p) | dishes |
 dishwasher (p) | cleaning supplies |
 sink (p) |  |
 microwave |
 waste basket (p) |  
 shelf (p) |  |
 refrigerator (p) |
 trash bin (p) | 
 desk (p) | fruits |
 bar (p) | drinks |
 tv stand (p) |
 cabinet (p) | foods |
 sofa (p) |
 seats |
"""
tools = """
def Navigate(target_location: str):
    \"\"\"Move the robot to a specified target location.\"\"\"

def FollowPerson(person_name: str):
    \"\"\"Follow a specific person standing in front of the robot as they move.
    Notes: Robot should navigate in front of person before  calling this function\"\"\"

def PickObject(object_name: str):
    \"\"\"Pick up a specified object nearby.\"\"\"

def PlaceObject(object_name: str, target_location: str):
    \"\"\"Place a carried object at the specified location.\"\"\"

def GreetPerson(person_name: str):
    \"\"\"Greet a specific person verbally or visually.\"\"\"

def AnswerQuestion(question: str):
    \"\"\"Answer a question posed by a person.\"\"\"

def IdentifyPerson(person_name: str):
    \"\"\"Provide identifying information about a known person or return their name.\"\"\"

def TellJoke(person_name: str):
    \"\"\"Tell a joke to a specified person.\"\"\"

def TellStatement(info_item: str):
    \"\"\"Share a fact or statement with a person.\"\"\"

def RecoginzeAction(person_name: str):
    \"\"\"Describe what action is the human performing, \"\"\"

def RecognizePosture(person_name: str):
    \"\"\"Describe what posture the human has, \"\"\"


def FindObject(object_name: str = None, object_type: str = None):
    \"\"\"
    Locate a specific object nearby.
    Parameters:
      - object_name (str): Name of the object to find.
      - object_type (str): Category or type of object to find.
    Note: To find an object in a location, navigate there first using Navigate().
    \"\"\"

def CountObjects(object_type: str):
    \"\"\"
    Count the number of objects of a specified type in fron of the robot

    Parameters:
      - object_type (str): Type/category of the object 
      
    Returns:
      - int: Number of objects found.
    \"\"\"

def ReportObjectAttribute(attribute: str, object_name: str = None):
    \"\"\"Describe an attribute  of a specified object.\"\"\"

def LocatePerson(person_name: str):
    \"\"\"Identify the current location of a specific person.
        Note: To locate a person in a location, navigate there first using Navigate().
    \"\"\"

def AskQuestionToPerson(person_name: str, question: str):
    \"\"\"Ask a specific question to a person and get their response.\"\"\"

def GuidePersonToLocation(person_name: str, target_location: str):
    \"\"\"Lead or escort a person from current location to the target location.\"\"\"

def IntroduceYourselfToPerson(person_name: str):
    \"\"\"Introduce yourself verbally or visually to a specified person.\"\"\"

def ReportNumberOfPeopleWithAttribute(attribute: str, location: str):
    \"\"\"Count how many people at a location match a given attribute .\"\"\"

def TellPersonInfo(person_name: str, info: str):
    \"\"\"Communicate specific information or statements to a given person.\"\"\"
def GiveObjectToPerson(person_location: str):
    '''
    Hand over the currently held object to a person on a specified location. If robot is not in that location navigate(location) must be called first
    Note: The robot should already be at the same location as the person before giving.
    '''
def ReportResult(info: str):
    '''Verbally report the result of a previous query .'''

    

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



def plan_command(command, model="koesn/mistral-7b-instruct", tools=tools, locations=locations_names):
    prompt = f"""
    You are a strict robot action planner. Only these actions exist exactly as defined:

    {tools}

    Known locations:
    {locations}

    Assumptions:
    - The robot always starts at "start_location" facing the user.
    - The user issuing the command is always at "start_location".
    - The word "me" in the command means the user at "start_location".
    - All required objects and people exist at expected locations.
    - Your output must always be a valid plan, no error handling needed.
    - Use exact Python syntax with double quotes for strings.
    - Output ONLY the plan variable in this exact format:

    plan = [
        # sequence of actions
    ]

    Important rules:
    - If the command includes "me" or implies the user, the robot must give the object to "user" at "start_location" using GiveObjectToPerson("user").
    - If the command names a different person or location as recipient, use GiveObjectToPerson(recipient) or GiveObject(location) accordingly.
    - Always navigate back to "start_location" after completing the main task unless the command explicitly states otherwise.
    - If the command involves reporting an attribute, usually a tell me action, answering a question, or stating a result (e.g., counting people), the robot must first gather the required information, then return to start_location to report it to the user.
    - The user is always at start_location, so there is no need to LocatePerson("user") or ask the user for help locating things.
    - Your plan must be a static sequence of actions, **no conditional statements, no if/else blocks, or branching logic inside the plan.**

    Examples:

    command = Get me a bowl from the kitchen
    plan = [
        Navigate("kitchen"),
        FindObject(object_name="bowl"),
        PickObject("bowl"),
        Navigate("start_location"),
        GiveObjectToPerson("user")
    ]

    command = Deliver the book on the table to Alex in the living_room
    plan = [
        Navigate("table"),
        FindObject(object_name="book"),
        PickObject("book"),
        Navigate("living_room"),
        GiveObjectToPerson("Alex"),
        Navigate("start_location")
    ]

    command = Tell me how many people in the bedroom are wearing orange sweaters
    plan = [
        Navigate("bedroom"),
        CountObjects(object_type="person wearing orange sweater"),
        Navigate("start_location"),
        ReportNumberOfPeopleWithAttribute("wearing orange sweaters")
    ]

    Now, provide ONLY the plan for this command:

    User command: {command}
    """

    messages = [
        {"role": "system", "content": "You are a strict robot action planner."},
        {"role": "user", "content": prompt}
    ]

    response = chat(model, messages=messages)
    plan = response["message"]["content"]
    print("Raw plan from model:", plan)
    return plan

    



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
