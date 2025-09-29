import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class CameraInferenceNode(Node):
    def __init__(self):
        super().__init__('camera_inference_node')
        self.bridge = CvBridge()
        self.model = YOLO("/home/strix-0/Yolo_models/best_segment_yolov8-seg.pt")   
        self.subscription = self.create_subscription(
            Image,
            '/dart_camera_front_right/pylon_ros2_camera_node_right/image',
            self.listener_callback,
            10
        )
        self.get_logger().info("Camera Inference Node started.")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image)  
            if isinstance(results, list):
                results = results[0]

            # Get boxes, masks, and classes
            boxes = results.boxes  # Bounding boxes
            masks = results.masks  # Segmentation masks
            class_names = results.names  # Class names
            confidences = results.boxes.conf  # Confidence scores for bounding boxes

            # Initialize frame to be the original if no detection
            annotated_frame = cv_image

            # Check if there are detections
            if len(boxes) == 0:
                self.get_logger().info("No detections in this frame")
                annotated_frame = cv_image 
            else:
                self.get_logger().info(f"Detections: {len(boxes)}")

                for i, box in enumerate(boxes):
                    confidence = confidences[i].item()  
                    if confidence > 0.94:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        class_id = int(box.cls.cpu().numpy())
                        class_name = class_names[class_id]
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box

                        # Add text: confidence and class name
                        text = f"{class_name} {confidence:.2f}" 
                        cv2.putText(annotated_frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Draw segmentation masks (if available and confidence is high)
                if masks is not None:
                    for i, mask in enumerate(masks):
                        mask_array = mask.data.cpu().numpy()  # Ensure it's a NumPy array
                        if mask_array.size > 0:  # Check if the mask is not empty
                            try:
                                # Resize the mask to match the original image size
                                mask_resized = cv2.resize(mask_array, (cv_image.shape[1], cv_image.shape[0]))  # Resize to match image size
                                mask_colored = np.zeros_like(cv_image)
                                mask_colored[mask_resized > 0] = [0, 255, 0]  # Green mask
                                annotated_frame = cv2.addWeighted(annotated_frame, 1.0, mask_colored, 0.5, 0)  # Overlay mask
                            except Exception as e:
                                self.get_logger().error(f"Error resizing mask: {e}")
                                continue  # Skip this mask if resizing failed


            cv2.imshow("YOLO Inference", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraInferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
