#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from face_recog.srv import RecognizeFace, RecognizeFaceRequest
from std_msgs.msg import String

def capture_image():
    cap = cv2.VideoCapture(0)  # Abre la cámara
    if not cap.isOpened():
        rospy.logerr("No se pudo abrir la cámara")
        return None
    
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        rospy.logerr("No se pudo capturar la imagen")
        return None
    
    return frame

def main():
    rospy.init_node('face_recognition_client')
    bridge = CvBridge()
    
    rospy.wait_for_service('recognize_face')
    rospy.wait_for_service('new_face')
    recognize_face_srv = rospy.ServiceProxy('recognize_face', RecognizeFace)
    new_face_srv = rospy.ServiceProxy('new_face', RecognizeFace)
    
    while not rospy.is_shutdown():
        mode = input("Ingresa 0 para nueva cara, 1 para reconocer: ")
        
        if mode not in ['0', '1']:
            print("Entrada inválida. Ingresa 0 o 1.")
            continue
        
        frame = capture_image()
        if frame is None:
            continue
        
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        if mode == '0':
            name = input("Ingresa el nombre para esta cara: ")
            req = RecognizeFaceRequest()
            req.in_.image_msgs.append(img_msg)
            req.Ids.ids.append(String(name))
            
            response = new_face_srv(req)
            print("Respuesta del servidor:", response.Ids.ids[0].data)
        
        elif mode == '1':
            req = RecognizeFaceRequest()
            req.in_.image_msgs.append(img_msg)
            
            response = recognize_face_srv(req)
            print("Persona reconocida:", response.Ids.ids[0].data)

if __name__ == "__main__":
    main()
