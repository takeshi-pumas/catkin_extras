#!/usr/bin/env python3
import rospy
import cv2
import os
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_classification.srv import Classify_dino, Classify_dinoResponse
from groundingdino.util.inference import load_model, predict, annotate
import clip
from PIL import Image

# Initialize CvBridge
bridge = CvBridge()

# Normalization values (same as GroundingDINO)
MEAN = [0.485, 0.456, 0.406]
STD = [0.229, 0.224, 0.225]

# Load CLIP Model
device = "cuda" if torch.cuda.is_available() else "cpu"
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

THRESHOLD = 0.3  # 🔹 CLIP Similarity threshold (adjust as needed)
MAX_RETRIES = 30  # 🔹 Prevents infinite loops
NUM_BOXES = 3  

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

def classify_with_clip(image_crop, prompt):
    """Classifies a cropped bounding box using CLIP."""
    pil_image = Image.fromarray(image_crop)
    image_clip = clip_preprocess(pil_image).unsqueeze(0).to(device)
    
    # Create both positive and negative prompts for contrast
    prompt_texts = [
        f"A photo of a {prompt}.",  # Positive class
        "A photo of something else."  # Negative class for contrast
    ]
    
    text_clip = clip.tokenize(prompt_texts).to(device)

    with torch.no_grad():
        image_features = clip_model.encode_image(image_clip)  # Extract image embedding
        text_features = clip_model.encode_text(text_clip)  # Extract text embeddings
        
        # Compute cosine similarity instead of softmax
        similarity = torch.nn.functional.cosine_similarity(image_features, text_features)

    similarity_score = similarity[0].item()  # Extract the similarity for the first text prompt
    return similarity_score


def handle_detection(req):
    """Handles incoming ROS service requests."""
    
    # Convert ROS image message to OpenCV format
    image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")
    image_source = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    processed_image = preprocess_image(image_source)

    # Extract prompt text (drink name)
    prompt_text = req.prompt.data.strip()
    rospy.loginfo(f"Processing request with drink prompt: {prompt_text}")

    # Get all bounding boxes (no limit)
    boxes, logits, phrases = predict(
        model=model,
        image=processed_image,
        caption="drink",
        box_threshold=0.4,  # Adjustable confidence threshold
        text_threshold=0.2
    )

    hi, we = image_source.shape[:2]

    # Convert bounding boxes from normalized values to absolute pixel values
    abs_boxes = []
    for box in boxes:
        abs_box = box * torch.tensor([we, hi, we, hi])
        abs_box = abs_box.numpy().astype("int")
        cx, cy, wd, ht = abs_box
        abs_boxes.append((cx, cy, wd, ht))

    if not abs_boxes:
        rospy.logwarn("No bounding boxes detected.")
        return Classify_dinoResponse(image=bridge.cv2_to_imgmsg(image_source, encoding="rgb8"), result=String(data="not found"))

    # Sort boxes by x_center (cx)
    abs_boxes.sort(key=lambda b: b[0])

    # Assign positions
    labeled_boxes = []
    for i, box in enumerate(abs_boxes):
        pos = "center"  # Default to center
        if i == 0:
            pos = "left"  # Leftmost box
        elif i == len(abs_boxes) - 1:
            pos = "right"  # Rightmost box
        
        labeled_boxes.append({"position": pos, "box": box})

    best_match = None
    best_similarity = 0.0

    for entry in labeled_boxes:
        pos = entry["position"]
        cx, cy, wd, ht = entry["box"]

        # Calculate bounding box coordinates
        x_min = cx - (wd // 2)
        x_max = cx + (wd // 2)
        y_min = cy - (ht // 2)
        y_max = cy + (ht // 2)

        cropped_image = image_source[y_min:y_max, x_min:x_max]  

        similarity_score = classify_with_clip(cropped_image, prompt_text)
        rospy.loginfo(f"Similarity score for {prompt_text} in {pos}: {similarity_score:.2f}")

        if similarity_score > best_similarity and similarity_score >= THRESHOLD:
            best_similarity = similarity_score
            best_match = pos

    annotated_frame = annotate(image_source, boxes=boxes, logits=logits, phrases=phrases)
    ros_annotated_image = bridge.cv2_to_imgmsg(annotated_frame, encoding="rgb8")

    return Classify_dinoResponse(image=ros_annotated_image, result=String(data=best_match or "not found"))


    

def grounding_dino_server():
    rospy.init_node('grounding_dino_server')
    rospy.Service('grounding_dino_detect', Classify_dino, handle_detection)
    rospy.loginfo("GroundingDINO service is running...")
    rospy.spin()

if __name__ == "__main__":
    grounding_dino_server()
