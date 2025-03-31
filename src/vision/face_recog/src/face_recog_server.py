#!/usr/bin/env python3

import os
import rospy
import numpy as np
# import cv2
import face_recognition
from rospkg import RosPack
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Bounding_Box
from face_recog.srv import FaceRecognition, FaceRecognitionResponse

### Inicialización global ###
rp = RosPack()
path_for_faces = os.path.join(rp.get_path('config_files'), 'faces_for_recognition')

bridge = CvBridge()
encodings = []
ids = []

### Cargar imágenes de referencia al iniciar ###
def load_known_faces():
    global encodings, ids
    encodings.clear()
    ids.clear()
    
    for person in os.listdir(path_for_faces):
        person_path = os.path.join(path_for_faces, person)
        if not os.path.isdir(person_path):
            continue  # Ignorar archivos que no sean carpetas

        for image_file in os.listdir(person_path):
            img_path = os.path.join(person_path, image_file)
            dataset_img = face_recognition.load_image_file(img_path)
            face_enc = face_recognition.face_encodings(dataset_img)

            if face_enc:
                encodings.append(face_enc[0])
                ids.append(person)

    ids = np.array(ids)  # Convertir a array numpy para optimizar búsqueda
    rospy.loginfo(f"Cargadas {len(encodings)} caras conocidas.")

### Reconocimiento de caras ###
def recognize_callback(req):
    """Reconoce caras en la imagen recibida"""
    if not encodings:
        load_known_faces()  # Cargar en caso de que no haya datos

    rospy.loginfo("Recibida imagen para reconocimiento.")

    # Convertir imagen de ROS a OpenCV
    image = bridge.imgmsg_to_cv2(req.input_img)
    face_locations = face_recognition.face_locations(image)

    if not face_locations:
        return FaceRecognitionResponse(
            name_response=[String("NO_FACE")], 
            features=[], 
            bounding_boxes=[]
        )

    rospy.loginfo(f"Se encontraron {len(face_locations)} caras.")

    face_encodings = face_recognition.face_encodings(image, face_locations)
    recognized_names = []
    bounding_boxes = []

    for i, enc in enumerate(face_encodings):
        matches = face_recognition.compare_faces(encodings, enc)
        if any(matches):
            best_match_index = np.argmax(matches)
            name = ids[best_match_index]
        else:
            name = "unknown"

        recognized_names.append(String(name))

        # Obtener coordenadas de la bounding box en el formato adecuado
        top, right, bottom, left = face_locations[i]
        bbox = Bounding_Box()
        bbox.xmin = left
        bbox.ymin = top
        bbox.xmax = right
        bbox.ymax = bottom
        bbox.Class = name
        # bbox.center.x = (left + right) / 2.0
        # bbox.center.y = (top + bottom) / 2.0
        # bbox.center.theta = 0.0 
        # bbox.size_x = right - left
        # bbox.size_y = bottom - top

        bounding_boxes.append(bbox)

    rospy.loginfo(f"Caras reconocidas: {[n.data for n in recognized_names]}")

    return FaceRecognitionResponse(
        name_response=recognized_names,
        features=[String("N/A")] * len(recognized_names),  # Placeholder para características
        bounding_boxes=bounding_boxes
    )

def train_face_callback(req):
    """
    Entrena una nueva cara desde la imagen recibida.
    Args:
        req: Solicitud del servicio que contiene la imagen y el nombre de la persona.
    """
    global encodings, ids  # Asegurar que modificamos las variables globales

    try:
        if not req.name_request:
            rospy.logwarn("No se proporcionó un nombre para el entrenamiento.")
            return FaceRecognitionResponse(
                name_response=[String("NO_NAME")],
                features=[],
                bounding_boxes=[]
            )

        person_name = req.name_request[0].data  # Extraer string real
        rospy.loginfo(f"Entrenando nueva cara para: {person_name}")

        # Convertir imagen ROS a OpenCV
        image = bridge.imgmsg_to_cv2(req.input_img, desired_encoding='rgb8')

        # Detectar caras en la imagen
        face_locations = face_recognition.face_locations(image)

        if not face_locations:
            rospy.logwarn("No se detectó ninguna cara en la imagen de entrenamiento.")
            return FaceRecognitionResponse(
                name_response=[String("NO_FACE")],
                features=[],
                bounding_boxes=[]
            )

        if len(face_locations) > 1:
            rospy.logwarn("Se detectaron múltiples caras en la imagen. Se usará la primera.")

        # Crear directorio de la persona si no existe
        person_dir = os.path.join(path_for_faces, person_name)
        os.makedirs(person_dir, exist_ok=True)

        # Extraer y codificar la cara detectada
        face_encoding = face_recognition.face_encodings(image, [face_locations[0]])[0]

        # Asegurar que encodings y ids sean listas antes de modificarlas
        if isinstance(encodings, np.ndarray):
            encodings = encodings.tolist()
        if isinstance(ids, np.ndarray):
            ids = ids.tolist()

        # Guardar encoding en memoria
        encodings.append(face_encoding)
        ids.append(person_name)

        # Guardar la imagen de la cara recortada
        top, right, bottom, left = face_locations[0]
        face_crop = image[top:bottom, left:right]  # Recortar la cara detectada

        timestamp = rospy.Time.now().to_sec()
        image_filename = f"{person_name}_{int(timestamp)}.jpg"
        image_path = os.path.join(person_dir, image_filename)

        import cv2
        cv2.imwrite(image_path, cv2.cvtColor(face_crop, cv2.COLOR_RGB2BGR))  # Guardar en formato correcto

        rospy.loginfo(f"Entrenamiento exitoso para {person_name}. Imagen guardada en {image_path}")

        # Crear bounding box para la respuesta
        bbox = Bounding_Box()
        bbox.xmin = left
        bbox.ymin = top
        bbox.xmax = right
        bbox.ymax = bottom
        bbox.Class = person_name

        return FaceRecognitionResponse(
            name_response=[String(person_name)],
            features=[String("N/A")],
            bounding_boxes=[bbox]
        )

    except Exception as e:
        rospy.logerr(f"Error entrenando la cara: {e}")
        return FaceRecognitionResponse(
            name_response=[String("ERROR")],
            features=[],
            bounding_boxes=[]
        )


### Inicialización del servidor ###
def classify_server():
    rospy.init_node("face_recognition_server")

    load_known_faces()


    recog_face_serv = "/recognize_face"
    rospy.Service(recog_face_serv, FaceRecognition, recognize_callback)

    # Training service
    train_face_serv = "/new_face"
    rospy.Service(train_face_serv, FaceRecognition, train_face_callback)

    rospy.loginfo("Face recognition services active.")
    rospy.loginfo(f"Recognition service: {recog_face_serv}")
    rospy.loginfo(f"Training service: {train_face_serv}")

    rospy.spin()

if __name__ == "__main__":
    classify_server()
