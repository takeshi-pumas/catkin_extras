#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import List
from sensor_msgs.msg import Image
from std_msgs.msg import String
from face_recog.srv import FaceRecognition, FaceRecognitionRequest
# import sys

# Inicializar ROS y la conversión de imágenes
# rospy.init_node("face_recognition_client")
bridge = CvBridge()
IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/rgb/image_raw"


def capture_frame(image_topic: str = IMAGE_TOPIC):
    """Espera y obtiene una imagen del sensor."""
    rospy.loginfo("Esperando una imagen...")
    image_msg = rospy.wait_for_message(image_topic, Image)
    return image_msg


def recognize_face(image: Image, timeout: float = 5.0) -> List[String]:
    """
    Send image to face recognition service with timeout retry logic
    
    Args:
        image: ROS Image message to process
        timeout: Maximum time to keep retrying if no face detected (seconds)
        
    Returns:
        List of detected face names or empty list if timeout reached
    """
    rospy.wait_for_service("/recognize_face")
    start_time = rospy.Time.now()
    
    try:
        recognize_service = rospy.ServiceProxy("/recognize_face", FaceRecognition)
        request = FaceRecognitionRequest()
        request.input_img = image

        while not rospy.is_shutdown():
            # Check timeout
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Face recognition timeout reached")
                return []

            response = recognize_service(request)
            
            # Check for valid response
            if response.name_response and response.name_response[0].data.lower() not in ["none", "no_face"]:
                # rospy.loginfo(f"Faces recognized: {', '.join([name.data for name in response.name_response])}")
                # cast the response to a list of str python string
                return [str(name.data) for name in response.name_response]
                # return response.name_response
                
            rospy.sleep(0.2)  # Wait before retrying
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return []


def train_face(image: Image, name: str):
    """Envía una imagen al servicio de entrenamiento."""
    rospy.wait_for_service("/new_face")
    try:
        train_service = rospy.ServiceProxy("/new_face", FaceRecognition)
        request = FaceRecognitionRequest()
        request.input_img = image  # Obtener la imagen más reciente
        if name:
            request.name_request = [String(name)]
        else: 
            raise NameError

        response = train_service(request)
        rospy.loginfo(f"Respuesta del entrenamiento: {response.name_response[0].data}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Error llamando al servicio: {e}")