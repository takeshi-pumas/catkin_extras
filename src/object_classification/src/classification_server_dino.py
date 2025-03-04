#!/usr/bin/env python3
import rospy
import cv2
import os
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from matplotlib import pyplot as plt
from geometry_msgs.msg import Polygon, Point32
from object_classification.srv import Classify_dino, Classify_dinoResponse, Classify_dinoRequest
from groundingdino.util.inference import load_model, predict, annotate

# Initialize CvBridge
bridge = CvBridge()

# Normalization values (same as GroundingDINO)
MEAN = [0.485, 0.456, 0.406]
STD = [0.229, 0.224, 0.225]

def preprocess_image(cv2_image, max_size=1333, stride=32, device="cuda"):
    """
    Convert an OpenCV image to the format expected by GroundingDINO.

    Args:
        cv2_image: OpenCV image in BGR format.
        max_size: Maximum size for resizing while keeping aspect ratio.
        stride: Ensure width and height are multiples of this value.
        device: 'cpu' or 'cuda'

    Returns:
        torch.Tensor: Normalized image tensor for the model.
    """
    # Convert BGR to RGB
    image_rgb = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)

    # Resize image while maintaining aspect ratio
    h, w = image_rgb.shape[:2]
    scale_factor = min(max_size / max(h, w), 1.0)  # Avoid upscaling
    new_h = int(h * scale_factor)
    new_w = int(w * scale_factor)

    # Ensure dimensions are multiples of stride
    new_h = int(round(new_h / stride) * stride)
    new_w = int(round(new_w / stride) * stride)

    image_resized = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    # Convert image to PyTorch tensor (C, H, W format)
    image_tensor = torch.tensor(image_resized, dtype=torch.float32).permute(2, 0, 1).squeeze(0).to(device)

    # Normalize pixel values (0-1 range)
    image_tensor = image_tensor / 255.0

    # Apply GroundingDINO normalization
    mean = torch.tensor(MEAN).view(3, 1, 1).to(device)
    std = torch.tensor(STD).view(3, 1, 1).to(device)
    image_tensor = (image_tensor - mean) / std

    return image_tensor



# Set base directory
BASE_DIR = os.path.expanduser("~/GroundingDINO")
CONFIG_PATH = os.path.join(BASE_DIR, "groundingdino/config/GroundingDINO_SwinT_OGC.py")
WEIGHTS_PATH = os.path.join(BASE_DIR, "weights", "groundingdino_swint_ogc.pth")

# Load GroundingDINO Model
rospy.loginfo("Loading GroundingDINO model...")
model = load_model(CONFIG_PATH, WEIGHTS_PATH)
rospy.loginfo("Model loaded successfully.")

def handle_detection(req):
    """Handles incoming ROS service requests."""
    try:
        # Convert ROS image message to OpenCV format
        image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")

        # Convert OpenCV image to the format expected by GroundingDINO
        image_source = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        processed_image = preprocess_image(image_source, device="cuda")  # Convert to tensor

        # Extract and clean the prompt string
        prompt_text = req.prompt.data.strip()
        rospy.loginfo(f"Processing request with prompt: {prompt_text}")

        # Run object detection
        boxes, logits, phrases = predict(
            model=model,
            image=processed_image,
            caption=prompt_text,
            box_threshold=0.2,
            text_threshold=0.2
        )

        # Annotate the image
        annotated_frame = annotate(image_source, boxes=boxes, logits=logits, phrases=phrases)

        # **Fix encoding issue**
        if annotated_frame is None or annotated_frame.size == 0:
            rospy.logerr("Error: Annotated image is empty!")
            return Classify_dinoResponse()

        ros_annotated_image = bridge.cv2_to_imgmsg(annotated_frame, encoding="rgb8")

        # Convert bounding boxes to ROS message format
        polygons = []
        for box in boxes:
            polygon = Polygon()
            polygon.points = [
                Point32(x=float(box[0]), y=float(box[1]), z=0),
                Point32(x=float(box[2]), y=float(box[1]), z=0),
                Point32(x=float(box[2]), y=float(box[3]), z=0),
                Point32(x=float(box[0]), y=float(box[3]), z=0)
            ]
            polygons.append(polygon)

        rospy.loginfo(f"Detected {len(polygons)} objects for prompt: {prompt_text}")

        # Return response with the processed image and bounding boxes
        return Classify_dinoResponse(image=ros_annotated_image, bounding_boxes=polygons)

    except Exception as e:
        rospy.logerr(f"Error processing request: {e}")
        return Classify_dinoResponse()


def grounding_dino_server():
    """Initializes the ROS service server."""
    rospy.init_node('grounding_dino_server')
    service = rospy.Service('grounding_dino_detect', Classify_dino, handle_detection)
    rospy.loginfo("GroundingDINO service is running...")
    rospy.spin()

if __name__ == "__main__":
    grounding_dino_server()
