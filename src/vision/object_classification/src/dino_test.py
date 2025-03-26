import cv2
import os
from groundingdino.util.inference import load_model, load_image, predict, annotate
import supervision as sv
from matplotlib import pyplot as plt

def process_image(image_name, text_prompt):
    # Set the base directory
    BASE_DIR = os.path.expanduser("~/Repositories/GroundingDINO")  # Update this if needed

    # Define the paths for configuration, model weights, and data
    CONFIG_PATH = os.path.join(BASE_DIR, "groundingdino/config/GroundingDINO_SwinT_OGC.py")  # Ensure it's YAML
    WEIGHTS_PATH = os.path.join(BASE_DIR, "weights", "groundingdino_swint_ogc.pth")
    IMAGE_DIR = os.getcwd()  # This makes it the current directory

    # Debug: Print paths
    print(f"CONFIG_PATH: {CONFIG_PATH}, exists: {os.path.isfile(CONFIG_PATH)}")
    print(f"WEIGHTS_PATH: {WEIGHTS_PATH}, exists: {os.path.isfile(WEIGHTS_PATH)}")

    # Ensure files exist
    if not os.path.isfile(CONFIG_PATH):
        raise FileNotFoundError(f"Configuration file not found: {CONFIG_PATH}")
    if not os.path.isfile(WEIGHTS_PATH):
        raise FileNotFoundError(f"Model weights not found: {WEIGHTS_PATH}")

    # Load the model
    model = load_model(CONFIG_PATH, WEIGHTS_PATH)

    # Define image path
    IMAGE_PATH = os.path.join(IMAGE_DIR, image_name)

    # Ensure the image exists
    if not os.path.isfile(IMAGE_PATH):
        raise FileNotFoundError(f"Error: Image '{IMAGE_PATH}' not found.")

    # Load the image
    image_source, image = load_image(IMAGE_PATH)

    # Convert BGR to RGB (if needed)
    image_rgb = cv2.cvtColor(image_source, cv2.COLOR_BGR2RGB)
    print(image_rgb.shape)
    # Set thresholds
    BOX_THRESHOLD = 0.2
    TEXT_THRESHOLD = 0.2

    print("Tensor shape:", image.shape)  # Debe ser (1, 3, 640, 640)
    print(f"Processed Image Shape: {image.shape}")
    print(f"Tensor Data Type: {image.dtype}")
    print(f"Device: {image.device}")

    # Run object detection
    boxes, logits, phrases = predict(
        model=model,
        image=image,
        caption=text_prompt,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD
    )

    # Annotate image
    annotated_frame = annotate(image_source=image_rgb, boxes=boxes, logits=logits, phrases=phrases)

    # Display the result
    plt.imshow(annotated_frame)
    plt.title(f"Prompt: {text_prompt}")
    plt.axis("off")  # Hide axis for better visualization
    plt.show()

if __name__ == "__main__":
    # Specify image and prompt
    IMAGE_NAME = "dog-2.jpeg"  # Change this to the correct image filename
    PROMPT = "salt"  # Change this to your desired prompt

    process_image(IMAGE_NAME, PROMPT)
