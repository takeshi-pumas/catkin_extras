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
        detect_drink = rospy.ServiceProxy('grounding_dino_detect', Classify_dino)
        response = detect_drink(ros_image, prompt_msg)

        print("Result:", response.result.data,"for drink:",prompt)
        if response.image is None or response.image.data == b'':
                print("Error: Received an empty image response!")
        else:
            debug_image = bridge.imgmsg_to_cv2(response.image, desired_encoding="rgb8")
            cv2.imwrite('debug_img.png', debug_image)
            #cv2.waitKey(1)  # Allow OpenCV to refresh window

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")



if __name__ == "__main__":
    rospy.init_node('grounding_dino_client')
    request_detection("drink_images/drink_image_6.jpeg", "coke")
