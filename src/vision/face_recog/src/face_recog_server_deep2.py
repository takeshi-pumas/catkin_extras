#!/usr/bin/env python3

import os
import cv2
import rospy
from cv_bridge import CvBridge
from rospkg import RosPack
from deepface import DeepFace
from face_recog.srv import FaceRecognition, FaceRecognitionResponse, FaceRecognitionRequest
from std_msgs.msg import String
from vision_msgs.msg import Bounding_Box

# Obtener la ruta de almacenamiento de rostros
rp = RosPack()
path_for_faces = os.path.join(rp.get_path('config_files'), 'faces_for_recognition')


def save_image(image, name):
    """ Guarda una imagen en la carpeta correspondiente a la ID. """
    person_path = os.path.join(path_for_faces, name)
    os.makedirs(person_path, exist_ok=True)  # Crea la carpeta si no existe

    count = len(os.listdir(person_path))
    filename = f"{name}{count}.jpg"
    filepath = os.path.join(person_path, filename)
    
    cv2.imwrite(filepath, image)
    print(f"Imagen guardada en {filepath}")


def new_face_callback(req: FaceRecognitionRequest):
    """ Registra nuevas caras en la base de datos. """
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(req.input_img)

    if len(req.name_request) != 1:
        raise ValueError("Solo se puede entrenar una imagen por solicitud.")
    
    name = req.name_request[0].data

    if os.path.exists(os.path.join(path_for_faces, name)):
        print(f"Nombre '{name}' ya existe. Agregando nueva imagen.")
    else:
        print(f"Nuevo nombre agregado: {name}")
    
    save_image(image, name)

    # Respuesta del servicio
    response = FaceRecognitionResponse()
    response.name_response.append(String(name))
    return response


def face_recognition_callback(req: FaceRecognitionRequest):
    """ Realiza reconocimiento facial comparando con la base de datos. """

    bridge = CvBridge()
    response = FaceRecognitionResponse()
    
    image = bridge.imgmsg_to_cv2(req.input_img)
    print(f"Procesando imagen de dimensiones: {image.shape}")

    try:
        extracted_faces = DeepFace.extract_faces(image)
        dfs = DeepFace.find(image, path_for_faces)

        # Verificar si dfs tiene resultados
        if dfs and len(dfs) > 0 and len(dfs[0]['identity']) > 1:
            names = [dfs[0]['identity'].iloc[1].split('/')[-2]]
            print(f"Rostros reconocidos: {names}")
            response.name_response = [String(name) for name in names]
            bb = Bounding_Box()
            bb.xmin = extracted_faces[0]['facial_area']['x']
            bb.ymin = extracted_faces[0]['facial_area']['y']
            bb.xmax = extracted_faces[0]['facial_area']['w'] + extracted_faces[0]['facial_area']['x']
            bb.ymax = extracted_faces[0]['facial_area']['h'] + extracted_faces[0]['facial_area']['y']
            response.bounding_boxes.append(bb)

        else:
            print("No se encontraron coincidencias.")
            response.name_response = [String("unknown")]
            bb = Bounding_Box()
            response.bounding_boxes.append(bb)

    except ValueError:
        print("No se detectó un rostro en la imagen.")
        response.name_response = [String("NO_FACE")]

    return response


def face_analysis_callback(req: FaceRecognitionRequest):
    """ Realiza análisis facial obteniendo edad, género, raza y emoción. """
    # print(f"Recibidas {len(req.input_img)} imágenes para análisis.")

    bridge = CvBridge()
    response = FaceRecognitionResponse()

    image = bridge.imgmsg_to_cv2(req.input_img)

    try:
        extracted_faces = DeepFace.extract_faces(image)
        objs = DeepFace.analyze(image, actions=['age', 'gender', 'race', 'emotion'])

        for attribute in ['dominant_gender', 'dominant_race', 'dominant_emotion']:
            response.features.append(String(objs[0][attribute]))
        response.features.append(String(str(objs[0]['age'])))

        # response.Rots.data = [
        #     extracted_faces[0]['facial_area']['y'],
        #     extracted_faces[0]['facial_area']['h'],
        #     extracted_faces[0]['facial_area']['w'],
        #     extracted_faces[0]['facial_area']['x']
        # ]

    except ValueError:
        print("No se detectó un rostro en la imagen.")
        response.name_response.append(String("NO_FACE"))
        # response.Ds.data.append(0.0)
        # response.Rots.data.append(0.0)

    return response


def classify_server():
    """ Inicializa los servicios de reconocimiento facial en ROS. """
    rospy.init_node('face_analysis_deep_available')

    new_face_srv = "/face_recognition/new_face"
    recog_face_srv = "/face_recognition/recognize_face"
    analyze_face_srv = "/face_recognition/analyze_face"

    rospy.Service(new_face_srv, FaceRecognition, new_face_callback)
    rospy.Service(recog_face_srv, FaceRecognition, face_recognition_callback)
    rospy.Service(analyze_face_srv, FaceRecognition, face_analysis_callback)

    print("Face Recognition Deep Server Available")
    print("Available services: \n")
    print(new_face_srv)
    print(recog_face_srv)
    print(analyze_face_srv)

    rospy.spin()


if __name__ == "__main__":
    classify_server()
