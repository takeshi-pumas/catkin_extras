#!/usr/bin/env python3
import rospy
import cv2
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_classification.srv import Classify_dino_receptionist, Classify_dino_receptionistResponse
from object_classification.srv import Classify, ClassifyResponse
from ultralytics import YOLO
import numpy as np

bridge = CvBridge()

# Carga modelo YOLOv8 (puede ser yolov8n.pt o tu modelo entrenado)
device = "cuda" if torch.cuda.is_available() else "cpu"
model_path = "/home/angel/ANGEL/Angel_YOLO/yolo_11.pt"
model = YOLO(model_path)
rospy.loginfo(f"YOLOv8 model loaded on {device}")

CONFIDENCE_THRESHOLD = 0.3  # Umbral para filtrar detecciones

def handle_detection(req):
    # Convertir imagen ROS a OpenCV BGR
    image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")

    # Ejecutar inferencia
    results = model(image)

    # Extraer detecciones: boxes, confidences
    boxes = []
    confidences = []
    for r in results:
        # r.boxes.xyxy es tensor Nx4 (x1,y1,x2,y2)
        # r.boxes.conf es tensor N
        # r.boxes.cls es tensor N (clase)
        for box, conf in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy()):
            if conf >= CONFIDENCE_THRESHOLD:
                boxes.append(box)  # [x1,y1,x2,y2]
                confidences.append(conf)

    if not boxes:
        rospy.logwarn("No detections with confidence above threshold.")
        return ClassifyResponse(result=String(data="not found"))

    # Calcular el centro x de cada caja para ordenar
    centers_x = [ (b[0]+b[2])/2 for b in boxes ]
    sorted_indices = np.argsort(centers_x)

    # Ordenar cajas y confiabilidades
    boxes = [boxes[i] for i in sorted_indices]
    confidences = [confidences[i] for i in sorted_indices]

    # Asignar posición a cada caja (left, center, right)
    labeled_positions = []
    for i in range(len(boxes)):
        if i == 0:
            pos = "left"
        elif i == len(boxes)-1:
            pos = "right"
        else:
            pos = "center"
        labeled_positions.append(pos)

    # Seleccionar la caja con mayor confianza
    max_conf_idx = np.argmax(confidences)
    best_position = labeled_positions[max_conf_idx]

    rospy.loginfo(f"Detection positions: {labeled_positions}")
    rospy.loginfo(f"Best detection at position: {best_position} with confidence {confidences[max_conf_idx]:.2f}")

    return ClassifyResponse(result=String(data=best_position))

def yolo_detection_server():
    rospy.init_node('classification_server')
    rospy.Service('classify', Classify, handle_detection)
    rospy.loginfo("YOLOv8 detection service is running...")
    rospy.spin()

if __name__ == "__main__":
    yolo_detection_server()
