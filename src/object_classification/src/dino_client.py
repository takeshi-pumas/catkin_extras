#!/usr/bin/env python3
import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_classification.srv import Classify_dino,Classify_dinoRequest,Classify_dinoResponse

def request_detection(image_path, prompt):
    bridge = CvBridge()

    # Verify the image path
    if not os.path.isfile(image_path):
        print(f"Error: Image file '{image_path}' not found.")
        return
    
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Unable to read image '{image_path}'.")
        return

    # Convert image to ROS format
    ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

    # Create a proper ROS String message
    prompt_msg = String()
    prompt_msg.data = prompt

    rospy.wait_for_service('grounding_dino_detect')
    try:
        detect_service = rospy.ServiceProxy('grounding_dino_detect', Classify_dino)
        response = detect_service(ros_image, prompt_msg)

        print("Bounding Boxes:", response.bounding_boxes)

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")



if __name__ == "__main__":
    rospy.init_node('grounding_dino_client')
    request_detection("dog-2.jpeg", "drink")
