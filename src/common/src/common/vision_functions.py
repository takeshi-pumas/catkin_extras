#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from face_recog.srv import FaceRecognition, FaceRecognitionRequest
import sys

# Inicializar ROS y la conversión de imágenes
# rospy.init_node("face_recognition_client")
bridge = CvBridge()
IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/rgb/image_raw"


def capture_frame(image_topic: str = IMAGE_TOPIC):
    """Espera y obtiene una imagen del sensor."""
    rospy.loginfo("Esperando una imagen...")
    image_msg = rospy.wait_for_message(image_topic, Image)
    return image_msg


def recognize_face(image: Image):
    """Envía una imagen al servicio de reconocimiento de caras."""
    rospy.wait_for_service("/recognize_face")
    try:
        recognize_service = rospy.ServiceProxy("/recognize_face", FaceRecognition)
        request = FaceRecognitionRequest()
        request.input_img = image  # Obtener la imagen más reciente

        response = recognize_service(request)
        rospy.loginfo(f"Caras reconocidas: {', '.join([name.data for name in response.name_response])}")
        return response.name_response
    except rospy.ServiceException as e:
        rospy.logerr(f"Error llamando al servicio: {e}")


def train_face(image: Image):
    """Envía una imagen al servicio de entrenamiento."""
    rospy.wait_for_service("/new_face")
    try:
        train_service = rospy.ServiceProxy("/new_face", FaceRecognition)
        request = FaceRecognitionRequest()
        request.input_img = image  # Obtener la imagen más reciente
        request.name_request = [String(input("Ingrese el nombre de la persona: "))]

        response = train_service(request)
        rospy.loginfo(f"Respuesta del entrenamiento: {response.name_response[0].data}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Error llamando al servicio: {e}")