import cv2
import numpy as np
from ultralytics import YOLO
import os

def calculate_bounding_box_area(video_path, model_path, confidence_threshold=0.96):
    # Load the YOLOv8 model
    model = YOLO(model_path)

    # Open the video file
    cap = cv2.VideoCapture(video_path)

    # Check if the video was successfully opened
    if not cap.isOpened():
        print("Error: Unable to open video.")
        return

    # Initialize the dictionary to store the highest area for each class
    areas = {
        'erc_0-erc-75-erc_90-erc-50': 0,
        'erc_50': 0,
        'erc_75': 0,
        'erc_90': 0
    }

    # Get video properties
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Run the YOLO model for object detection and segmentation
        results = model(frame)

        # Process detections
        for result in results:
            boxes = result.boxes  # Get the bounding boxes of detected objects
            masks = result.masks  # Get the segmentation masks of detected objects

            # Loop through each detected box and its corresponding mask
            for i, box in enumerate(boxes):
                # Unpack the bounding box and other details from the result
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  # Extract coordinates from box
                conf = box.conf[0].cpu().numpy()  # Confidence score
                cls = box.cls[0].cpu().numpy()  # Class ID

                # Get the class label (i.e., ERC clip type)
                class_name = model.names[int(cls)]  # Get the class label from the class ID

                # Log the details for each detection
                print(f"Frame {frame_count}, Class: {class_name}, Conf: {conf:.2f}, "
                      f"Box: ({x1}, {y1}), ({x2}, {y2}), Area: {(x2 - x1) * (y2 - y1):.2f} pixels^2")

                # Check if the confidence is above the threshold
                if conf < confidence_threshold:
                    continue  # Skip if the confidence is too low

                # Only process the bounding boxes for the specified ERC types
                if class_name in ['erc_0-erc-75-erc_90-erc-50', 'erc_50', 'erc_75', 'erc_90']:
                    # Calculate the bounding box area
                    bbox_area = (x2 - x1) * (y2 - y1)

                    # Update the stored maximum area for that class if the current area is larger
                    if bbox_area > areas[class_name]:
                        areas[class_name] = bbox_area

        frame_count += 1

    # Release the video capture object
    cap.release()

    # Print the maximum bounding box area for each class
    print(f"Processed {frame_count} frames.")
    for class_name, max_area in areas.items():
        print(f"Maximum Bounding Box Area for {class_name}: {max_area:.2f} pixels^2")

# Example usage
video_path = "/home/strix-0/new_wenglor_ws/camera_record_20250705_184320.mp4"  # Replace with your input video file path
model_path = "/home/strix-0/Yolo_models/best_segment_yolov8-seg.pt"  # Replace with your trained YOLOv8 model path
calculate_bounding_box_area(video_path, model_path)
