#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from face_recog.srv import FaceRecognition, FaceRecognitionRequest
from std_msgs.msg import String

class FaceRecognitionClient:
    def __init__(self):
        rospy.init_node("face_recognition_client", anonymous=True)
        self.bridge = CvBridge()

        new_face_srv = "/face_recognition/new_face"
        recog_face_srv = "/face_recognition/recognize_face"
        analyze_face_srv = "/face_recognition/analyze_face"

        # Esperar a que los servicios estén disponibles
        rospy.wait_for_service(new_face_srv)
        rospy.wait_for_service(recog_face_srv)
        rospy.wait_for_service(analyze_face_srv)

        # Conectar con los servicios
        self.new_face_srv = rospy.ServiceProxy(new_face_srv, FaceRecognition)
        self.recognize_face_srv = rospy.ServiceProxy(recog_face_srv, FaceRecognition)
        self.analyze_face_srv = rospy.ServiceProxy(analyze_face_srv, FaceRecognition)


    def capture_image(self):
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

    def register_face(self, person_name):
        """Envía una imagen para registrar una nueva cara en la base de datos."""
        req = FaceRecognitionRequest()
        img_msg = self.bridge.cv2_to_imgmsg(self.capture_image(), encoding="rgb8")
        req.input_img = img_msg
        req.name_request = [String(person_name)]  # Cambiado a lista de String

        try:
            response = self.new_face_srv(req)
            print(f"✅ Registro exitoso: {response.name_response[0]}")
        except rospy.ServiceException as e:
            print(f"❌ Error en el servicio: {e}")

    def recognize_face(self):
        """Envía una imagen para identificar una cara."""
        req = FaceRecognitionRequest()
        img_msg = self.bridge.cv2_to_imgmsg(self.capture_image(), encoding="rgb8")
        req.input_img = img_msg

        try:
            response = self.recognize_face_srv(req)
            if response.name_response:
                print(f"✅ Persona reconocida: {response.name_response[0].data}")
            else:
                print("❌ No se encontró una coincidencia.")
        except rospy.ServiceException as e:
            print(f"❌ Error en el servicio: {e}")

    def analyze_face(self):
        """Envía una imagen para analizar la cara (edad, género, raza y emoción)."""
        req = FaceRecognitionRequest()
        img_msg = self.bridge.cv2_to_imgmsg(self.capture_image(), encoding="rgb8")
        req.input_img = img_msg

        try:
            response = self.analyze_face_srv(req)

            if response.features:
                print(f"✅ Género: {response.features[0].data}")
                print(f"✅ Raza: {response.features[1].data}")
                print(f"✅ Emoción: {response.features[2].data}")
                print(f"✅ Edad: {response.features[3].data}")
            else:
                print("❌ No se detectó un rostro.")
        except rospy.ServiceException as e:
            print(f"❌ Error en el servicio: {e}")


if __name__ == "__main__":
    client = FaceRecognitionClient()

    while not rospy.is_shutdown():
        # Opciones del usuario
        print("\n1. Registrar nueva cara")
        print("2. Reconocer cara")
        print("3. Analizar cara")
        option = input("Selecciona una opción (1-3): ")

        if option == "1":
            person_name = input("Nombre de la persona: ")
            client.register_face(person_name)
        elif option == "2":
            client.recognize_face()
        elif option == "3":
            client.analyze_face()
        else:
            print("❌ Opción inválida.")

