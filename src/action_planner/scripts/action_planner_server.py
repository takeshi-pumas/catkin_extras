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

######################################################################################
class RGB:
    def __init__(self):
        # Get the topic name from the parameter server
        self.image_topic = rospy.get_param('/image_topic', '/usb_cam/image_raw')  # Default to /hsrb/head_rgbd_sensor/rgb/image_rect_color
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
def plan_command(command):
    actions=read_yaml('actions.yaml')
    known_locs=read_yaml('known_locs_gpsr.yaml')
    ######################################################################################        
    rospack = rospkg.RosPack()                                                                                  # THIS CAN BE YAMLD TODO
    file_path=rospack.get_path('action_planner')+'/context_files/examples.txt'
    with open(file_path, 'r') as file:
        examples = file.read()
    ######################################################################################        
    print('Planning... wait for a few seconds ')
    messages = [
    {
          "role": "tool",
           "content": f"You are a robot with a limited action set described here: {actions}. Here are some examples of sequences of actions to solve commands: {examples}. User requests: {command}. Using ONLY the provided action set and known locations {known_locs}, generate a structured action plan that follows the sequence of tasks required to accomplish the goal. Ensure the output is in YAML format and includes specific actions from the available action set (e.g., navigate, locate_object, bring_object). All actions and parameters must be formatted in snake_case."
        },
    ]
    response = chat('mistral', messages=messages)
    print(response['message']['content'])
    messages = [
        {"role":"tool",
         "content":response['message']['content'] },
        {"role": "tool",
          "content": f" Great answer can you give it a format like this one plan=[just name_of_action(parameter), name_of_action2(parameter), etc], parameters must follow the snake_case convention"    
        },
    ]
    response2 = chat('mistral', messages=messages)  
    print(response2['message']['content'])
    return response2['message']['content']
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
    rospy.loginfo(f"Received: {req.command}")
    response = String()  # Create a new String message
    if len(req.command.data)!=0:
        if req.command.data[-1]=='?':# Expected strin in request for question qu: What question will be answered?
            print (f'question: {req.command.data}')
            answer=answer_question(req.command.data)
            response.data=answer
    else:
        qr_text=wait_for_qr(req.timeout)
        print(qr_text)
        if qr_text != "time out":
            command=qr_text
            plan=plan_command(command)
            corr_plan = check_format(plan)
            # Find the start and end of the sequence
            start = corr_plan.find("[")
            end =   corr_plan.find("]")
            # Extract the content inside the brackets
            if start != -1 and end != -1:
                sequence = corr_plan[start + 1:end].strip()  # Remove the brackets
                actions = [action.strip().strip('"') for action in sequence.split(",")]
                print(actions)
            else:
                print("No sequence found.")
            out_plan=', '.join(actions)
            out_plan
            response.data = out_plan
            print(response.data)
        else:
            print (' Qr Timed OuT')
            response.data ='timed out'
    return ActionPlannerResponse(response)

if __name__ == "__main__":
    global rgb
    rospy.init_node('action_planner_server')
    
    # Load the parameter for the image topic
    rospy.loginfo("Waiting for parameter 'image_topic'...")
    if rospy.has_param('~image_topic'):
        rospy.loginfo(f"Using image topic: {rospy.get_param('~image_topic')}")
    else:
        rospy.logwarn("Parameter 'image_topic' not found. Using default '/usb_cam/image_raw'.")

    service = rospy.Service('action_planner', ActionPlanner, action_planner)
    rgb = RGB()
    rospy.loginfo("Action Planner Service Ready")
    rospy.spin()
