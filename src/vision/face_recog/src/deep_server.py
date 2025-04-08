#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
from face_recog.srv import FaceRecognition, FaceRecognitionResponse, FaceRecognitionRequest
from deepface import DeepFace

bridge = CvBridge()

def face_analysis_callback(req: FaceRecognitionRequest):
    """ Realiza análisis facial obteniendo edad, género, raza y emoción. """
    
    response = FaceRecognitionResponse()

    try:
        # Convertir la imagen ROS a formato OpenCV
        image = bridge.imgmsg_to_cv2(req.input_img)

        # Extraer los rostros de la imagen
        extracted_faces = DeepFace.extract_faces(image, enforce_detection=False)

        # Si no se detectan rostros, devolver 'NO_FACE'
        if not extracted_faces:
            rospy.logwarn("No se detectaron rostros.")
            response.name_response.append(String("NO_FACE"))
            return response

        # Realizar análisis en cada rostro extraído
        objs = DeepFace.analyze(image, actions=['age', 'gender', 'race', 'emotion'])

        # Añadir las características de cada rostro a la respuesta
        for obj in objs:
            response.features.append(String(obj['dominant_gender']))
            response.features.append(String(obj['dominant_race']))
            response.features.append(String(obj['dominant_emotion']))
            response.features.append(String(str(obj['age'])))

        # Si no hay solicitud de nombres, no añadimos nada a name_response
        # Si lo deseas, podemos devolver algo específico como "UNKNOWN"
        for _ in objs:
            response.name_response.append(String("UNKNOWN"))

    except Exception as e:
        rospy.logerr(f"Error en análisis facial: {e}")
        response.name_response.append(String("ERROR"))

    return response


def main():
    rospy.init_node('face_analysis_server')
    rospy.Service("/face_recognition/analyze_face", FaceRecognition, face_analysis_callback)
    rospy.loginfo("Face Analysis Deep Server listo")
    rospy.spin()


if __name__ == "__main__":
    main()
