#!/usr/bin/env python3
import rospy
import cv2
import os
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_classification.srv import Classify_dino_receptionist, Classify_dino_receptionistResponse
from groundingdino.util.inference import load_model, predict, annotate
import clip
from PIL import Image

# Initialize CvBridge
bridge = CvBridge()

# Normalization values (same as GroundingDINO)
MEAN = [0.485, 0.456, 0.406]
STD = [0.229, 0.224, 0.225]

# Load CLIP Model
device = "cpu"
clip_model, clip_preprocess = clip.load("ViT-B/32", device=device)  
rospy.loginfo("CLIP model loaded successfully.")

# Set base directory
BASE_DIR = os.path.expanduser("~/GroundingDINO")
CONFIG_PATH = os.path.join(BASE_DIR, "groundingdino/config/GroundingDINO_SwinT_OGC.py")
WEIGHTS_PATH = os.path.join(BASE_DIR, "weights", "groundingdino_swint_ogc.pth")

# Load GroundingDINO Model
rospy.loginfo("Loading GroundingDINO model...")
model = load_model(CONFIG_PATH, WEIGHTS_PATH)
rospy.loginfo("Model loaded successfully.")

THRESHOLD = 0.28  # CLIP Similarity threshold (adjust as needed)

# 🔹 Diccionario de descripciones personalizadas
prompt_dict = {
    "coke": "A coke's bottle with a red label.",
    "coffee": "A cup of hot coffee.",
    "water": "A bottle of water with a sky blue label.",
    "juice": "A white bottle of mango juice.",
    "lipton": "A bottle with lipton label and amber drink."
}

def preprocess_image(cv2_image, max_size=1333, stride=32):
    """Convert an OpenCV image to the format expected by GroundingDINO."""
    image_rgb = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
    h, w = image_rgb.shape[:2]
    scale_factor = min(max_size / max(h, w), 1.0)
    new_h, new_w = int(h * scale_factor), int(w * scale_factor)
    new_h, new_w = int(round(new_h / stride) * stride), int(round(new_w / stride) * stride)
    image_resized = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    image_tensor = torch.tensor(image_resized, dtype=torch.float32).permute(2, 0, 1).to(device) / 255.0
    mean = torch.tensor(MEAN).view(3, 1, 1).to(device)
    std = torch.tensor(STD).view(3, 1, 1).to(device)
    return (image_tensor - mean) / std

def classify_with_clip(image_crop, prompt_key):
    """Classifies a cropped bounding box using CLIP with a prompt from prompt_dict."""
    pil_image = Image.fromarray(image_crop)
    image_clip = clip_preprocess(pil_image).unsqueeze(0).to(device)
    
    # Obtener la descripción del diccionario si existe
    prompt_text = prompt_dict.get(prompt_key, f"A photo of a {prompt_key}.")
    print(prompt_text)
    
    # Crear prompts para contraste
    prompt_texts = [
        prompt_text,  # Clase positiva
        "A photo of something else."  # Clase negativa para contraste
    ]
    
    text_clip = clip.tokenize(prompt_texts).to(device)

    with torch.no_grad():
        image_features = clip_model.encode_image(image_clip)  # Extraer embedding de imagen
        text_features = clip_model.encode_text(text_clip)  # Extraer embedding de texto
        
        # Calcular similitud coseno
        similarity = torch.nn.functional.cosine_similarity(image_features, text_features)

    similarity_score = similarity[0].item()  # Obtener la similitud para el primer prompt
    return similarity_score

def handle_detection(req):
    """Handles incoming ROS service requests."""
    
    # Convertir mensaje ROS a imagen OpenCV
    image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")
    image_source = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    processed_image = preprocess_image(image_source)

    # Extraer texto del prompt y verificar en el diccionario
    prompt = req.prompt.data.strip()
    if prompt not in prompt_dict:
        rospy.logwarn(f"Prompt '{prompt}' not found in dictionary. Using default.")
    
    rospy.loginfo(f"Processing request with prompt: {prompt}")

    # Obtener todas las bounding boxes
    boxes, logits, phrases = predict(
        model=model,
        image=processed_image,
        caption="drink",
        box_threshold=0.3,  # Umbral ajustable
        text_threshold=0.2
    )

    hi, we = image_source.shape[:2]

    # Convertir bounding boxes a valores absolutos en píxeles
    abs_boxes = []
    for box in boxes:
        abs_box = box * torch.tensor([we, hi, we, hi])
        abs_box = abs_box.numpy().astype("int")
        cx, cy, wd, ht = abs_box
        abs_boxes.append((cx, cy, wd, ht))

    if not abs_boxes:
        rospy.logwarn("No bounding boxes detected.")
        return Classify_dino_receptionistResponse(image=bridge.cv2_to_imgmsg(image_source, encoding="rgb8"), result=String(data="not found"))

    # Ordenar bounding boxes por su centro x
    abs_boxes.sort(key=lambda b: b[0])

    # Asignar posiciones
    labeled_boxes = []
    for i, box in enumerate(abs_boxes):
        pos = "center"  # Por defecto "center"
        if i == 0:
            pos = "left"  # Más a la izquierda
        elif i == len(abs_boxes) - 1:
            pos = "right"  # Más a la derecha
        
        labeled_boxes.append({"position": pos, "box": box})

    best_match = None
    best_similarity = 0.0

    for entry in labeled_boxes:
        pos = entry["position"]
        cx, cy, wd, ht = entry["box"]

        # Calcular coordenadas del bounding box
        x_min = cx - (wd // 2)
        x_max = cx + (wd // 2)
        y_min = cy - (ht // 2)
        y_max = cy + (ht // 2)

        cropped_image = image_source[y_min:y_max, x_min:x_max]  

        similarity_score = classify_with_clip(cropped_image, prompt)
        rospy.loginfo(f"Similarity score for {prompt} in {pos}: {similarity_score:.2f}")

        if similarity_score > best_similarity and similarity_score >= THRESHOLD:
            best_similarity = similarity_score
            best_match = pos

    return Classify_dino_receptionistResponse(result=String(data=best_match or "not found"))


def grounding_dino_server():
    rospy.init_node('grounding_dino_server')
    rospy.Service('grounding_dino_detect', Classify_dino_receptionist, handle_detection)
    rospy.loginfo("GroundingDINO service is running...")
    rospy.spin()

if __name__ == "__main__":
    grounding_dino_server()
