import cv2
import numpy as np
from ultralytics import YOLO
import os
from pathlib import Path

# Paths to images and model
images_path = "./lanefotodata"
model_path = "/home/merts/ros2_ws/src/simulasyon_2024/scripts/lane.pt"
output_dir = "./deneme_mark"

# Create the output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

# Load the model
model = YOLO(model_path)

# Get the list of images
images = os.listdir(images_path)

# Process each image
for idx, img_name in enumerate(images):
    # Load an image
    image_path = os.path.join(images_path, img_name)
    image = cv2.imread(image_path)

    # Process the image with the model
    results = model(image)
    masks = results[0].masks  # Model sonuçlarından maskeleri al

    b_mask = np.zeros(image.shape[:2], dtype=np.uint8)

    # Check if there are masks and process each one
    if masks is not None:
        for i, mask in enumerate(masks):
            # Convert mask to binary image
            binary_mask = mask.data.cpu().numpy().astype(np.uint8) * 255

            # Remove the first dimension if it exists
            if len(binary_mask.shape) == 3 and binary_mask.shape[0] == 1:
                binary_mask = binary_mask.squeeze(0)

            if binary_mask is not None and binary_mask.size > 0:
                if binary_mask.shape != b_mask.shape:
                    binary_mask = cv2.resize(binary_mask, (b_mask.shape[1], b_mask.shape[0]))

                b_mask = cv2.bitwise_or(b_mask, binary_mask)

    # Save the mask to the output directory
    output_path = os.path.join(output_dir, f"mask_{idx}.png")
    cv2.imwrite(output_path, b_mask)

    print(f"Saved mask to: {output_path}")

# Optional: Display the last processed mask
cv2.imshow("deneme", b_mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
