#!/usr/bin/env python3
import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_classification.srv import Classify_dino, Classify_dinoRequest, Classify_dinoResponse

def request_detection(ros_image, prompt):
    bridge = CvBridge()

    # Convert image to ROS format (already in ROS format here)
    prompt_msg = String()
    prompt_msg.data = prompt

    rospy.wait_for_service('grounding_dino_detect')
    try:
        detect_drink = rospy.ServiceProxy('grounding_dino_detect', Classify_dino)
        response = detect_drink(ros_image, prompt_msg)

        print("Result:", response.result.data, "for drink:", prompt)
        if response.image is None or response.image.data == b'':
            print("Error: Received an empty image response!")
        else:
            debug_image = bridge.imgmsg_to_cv2(response.image, desired_encoding="rgb8")
            cv2.imwrite('debug_img.png', debug_image)
            #cv2.waitKey(1)  # Allow OpenCV to refresh window

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert the ROS Image to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo("Image received successfully!")

        # Here, specify the prompt (you can customize this)
        prompt = "coke"
        
        # Call the request_detection function with the image and prompt
        request_detection(msg, prompt)

    except Exception as e:
        rospy.logerr("Error while processing the image: %s", str(e))

def image_listener():
    rospy.init_node('image_listener_client', anonymous=True)

    # Subscribe to the image topic
    rospy.Subscriber('/hsrb/head_r_stereo_camera/image_raw', Image, image_callback)

    rospy.spin()

if __name__ == "__main__":
    image_listener()
