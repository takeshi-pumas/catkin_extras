#!/usr/bin/env python3

from cv_bridge import CvBridge
from face_recog.msg import *
from face_recog.srv import *
# from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import rospy
import numpy as np
# import tf
# import tf2_ros
import face_recognition
import cv2
import os
from rospkg import RosPack


### Inicialización global ###
rp = RosPack()
path_for_faces = os.path.join(rp.get_path('config_files'), 'faces_for_recognition')

bridge = CvBridge()
encodings = []
ids = []

### Carga de imágenes de referencia al iniciar ###
def load_known_faces():
    global encodings, ids
    encodings.clear()
    ids.clear()
    
    for person in os.listdir(path_for_faces):
        if person.endswith('.pkl'):
            continue

        person_path = os.path.join(path_for_faces, person)
        for image_file in os.listdir(person_path):
            img_path = os.path.join(person_path, image_file)
            dataset_img = face_recognition.load_image_file(img_path)
            face_enc = face_recognition.face_encodings(dataset_img)

            if face_enc:
                encodings.append(face_enc[0])
                ids.append(person)

    ids = np.asarray(ids)  # Conversion a array numpy para optimizar búsqueda

load_known_faces()


### Conversión de coordenadas ###
def rect2sph(rectcoords):
    x, y, z = rectcoords
    r = np.sqrt(x**2 + y**2 + z**2)
    th = np.arctan2(y, x)
    phi = np.arctan2(np.sqrt(x**2 + y**2), z)
    return np.array([r, th, phi])


def sph2rect(sphrcoords):
    r, th, phi = sphrcoords
    x = r * np.sin(phi) * np.cos(th)
    y = r * np.sin(phi) * np.sin(th)
    z = r * np.cos(phi)
    return np.array([x, y, z])


### Guardar nuevas imágenes ###
def new_face_callback(req):
    """Almacena nuevas imágenes de caras en la base de datos"""
    print(f'Got {len(req.in_.image_msgs)} images to train new ID')
    print(f'Got {len(req.Ids.ids)} names')

    for i, img_msg in enumerate(req.in_.image_msgs):
        # image = cv2.cvtColor(bridge.imgmsg_to_cv2(img_msg), cv2.COLOR_BGR2RGB)
        image = bridge.imgmsg_to_cv2(img_msg)
        face_locations = face_recognition.face_locations(image)

        if len(face_locations) == 1:
            person_name = req.Ids.ids[i].data
            person_path = os.path.join(path_for_faces, person_name)

            if not os.path.exists(person_path):
                os.mkdir(person_path)

            img_count = len(os.listdir(person_path))
            img_filename = f"{person_name}{img_count}.jpg"
            cv2.imwrite(os.path.join(person_path, img_filename), image)

            message = f'Trained new ID: {person_name}'
            load_known_faces() # Recargar los datos de caras conocidas
            print(message)
        else:
            message = f'Image rejected: {len(face_locations)} faces detected'
            print(message)

    return RecognizeFaceResponse(Floats([0.0, 0.0]), Floats([1, 1]), Strings([String(message)]))


### Reconocimiento de caras ###
def recognize_callback(req):
    """Reconoce caras en imágenes recibidas"""
    if not encodings:
        load_known_faces()  # Cargar en caso de que no haya datos

    print(f'Got {len(req.in_.image_msgs)} images')

    images = [bridge.imgmsg_to_cv2(img_msg) for img_msg in req.in_.image_msgs]

    for image in images:
        face_locations = face_recognition.face_locations(image)
        if not face_locations:
            return RecognizeFaceResponse(Floats([0.0]), Floats([0.0]), Strings([String('NO_FACE')]))

        print(f'Faces found: {len(face_locations)}')

        face_encodings = face_recognition.face_encodings(image, face_locations)
        names = [ids[np.argmax(face_recognition.compare_faces(encodings, enc))] if any(face_recognition.compare_faces(encodings, enc)) else 'unknown' for enc in face_encodings]

        print(names)

        return RecognizeFaceResponse(Floats([0.0]), Floats([0.0]), Strings([String(name) for name in names]))


### Inicialización del servidor ###
def classify_server():
    rospy.init_node('face_recognition_server')

    recog_face_serv = "/recognize_face"
    train_new_face_serv = "/new_face"

    rospy.Service(recog_face_serv, RecognizeFace, recognize_callback)
    rospy.Service(train_new_face_serv, RecognizeFace, new_face_callback)

    rospy.loginfo("Face Recognition service available")
    rospy.loginfo(f"Service: {recog_face_serv} available")
    rospy.loginfo(f"Service: {train_new_face_serv} available")

    rospy.spin()

if __name__ == "__main__":
    classify_server()
