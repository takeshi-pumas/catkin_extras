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
######################################################################################
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


def plan_command(command, model='mistral'):
    # Load necessary data
    actions=read_yaml('actions.yaml')
    known_locs=read_yaml('known_locs_gpsr.yaml')
    ######################################################################################        
    rospack = rospkg.RosPack()                                                                                  # THIS CAN BE YAMLD TODO
    file_path=rospack.get_path('action_planner')+'/context_files/examples.txt'
    with open(file_path, 'r') as file:
        examples = file.read()
    ######################################################################################        
    print('Planning... wait for a few seconds ')
  
    prompt_1 = (
        f"You are a robot with a limited action set described here: {actions}. "
        #f"Here are some examples of sequences of actions to solve commands: {examples}. "
        f"User requests: {command}. Using ONLY the provided action set and known locations {known_locs}, "
        f"generate a structured action plan that follows the sequence of tasks required to accomplish the goal. "
        f"Ensure the output is in YAML format and includes specific actions from the available action set "
        f"(e.g., navigate, locate_object, bring_object). All actions and parameters must be formatted in snake_case."
        f"Assume all persons and objects can move, so unless directly mentioned in {known_locs} or in {command} assume location is unknown, and you should ask user for this info ( this is an action AskUser)"
        f"Do NOT add any assumptions or extra entities not mentioned in the command."
    )

    messages = [{"role": "tool", "content": prompt_1}]
    #response = chat('mistral', messages=messages)
    response = chat(model, messages=messages)
    plan_yaml = response['message']['content']
    print(plan_yaml)

    # Step 2: Reformat the plan into a list format
    prompt_2 = (
        f"Based on the previous YAML action plan:\n{plan_yaml}\n\n"
        f"Reformat the actions into this format: "
        f"plan=[name_of_action(parameter), name_of_action2(parameter), etc]. "
        f"Ensure all action parameters follow snake_case convention and the output is concise."
    )

    messages = [
        {"role": "tool", "content": plan_yaml},
        {"role": "tool", "content": prompt_2},
    ]
    response2 = chat(model, messages=messages)
    plan_list = response2['message']['content']
    print(plan_list)

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
    response = String()  # Create a new String message
    if len(req.command.data)!=0:
        if req.command.data[-1]=='?':#  What question will be answered?
            print (f'question: {req.command.data}')
            answer=answer_question(req.command.data)
            response.data=answer
    else:
        qr_text=wait_for_qr(req.timeout)
        print(qr_text)
        if qr_text != "time out":
            command=qr_text
            plan=plan_command(command)
            check_plan=fact_check(plan)

            #corr_plan = check_format(plan)
            # Find the start and end of the sequence
            #start = corr_plan.find("[")
            #end =   corr_plan.find("]")
            ## Extract the content inside the brackets
            #if start != -1 and end != -1:
            #    sequence = corr_plan[start + 1:end].strip()  # Remove the brackets
            #    actions = [action.strip().strip('"') for action in sequence.split(",")]
            #    print(actions)
            #else:
            #    print("No sequence found.")
            #out_plan=', '.join(actions)
            #out_plan
            if check_plan=='continue':
                print ("Plan accepted. executing")
                response.data = plan
            else: 
                print ( "more info is needed.Ask user")
                response.data = check_plan 
            print(f"{response.data}\n \n")
        else:
            print (' Qr Timed OuT')
            response.data ='timed out'
    return ActionPlannerResponse(response)

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
