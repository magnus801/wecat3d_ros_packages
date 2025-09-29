import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import time

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Initialize a ROS2 subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/dart_camera_front_right/pylon_ros2_camera_node_right/image/image_raw',
            self.listener_callback,
            10
        )
        self.subscription
        self.bridge = CvBridge()
        self.model = YOLO("/home/strix-0/Yolo_models/best_under_over_fir.pt")  # Load YOLOv11n-seg model (nano segmentation model)
        self.get_logger().info("Camera Subscriber Node Initialized!")

        # Benchmarking variables
        self.frame_times = []
        self.start_time = time.time()

        # Frame count to print stats periodically (e.g., every 100 frames)
        self.frame_count = 0
        self.shutdown_flag = False

    def listener_callback(self, msg):
        try:
            # Start time for frame processing
            frame_start_time = time.perf_counter()

            # Convert ROS2 Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run the YOLO model for segmentation (inference)
            results = self.model(cv_image)

            # Iterate through the results and filter by confidence
            annotated_frame = cv_image.copy()  # Make a copy of the original frame to annotate

            for result in results:
                boxes = result.boxes

                # Filter boxes with confidence greater than 0.98
                high_confidence_boxes = boxes[boxes.conf > 0.97]

                # Process only high-confidence boxes
                for box in high_confidence_boxes:
                    # Check the shape of box.conf before indexing
                    if len(box.conf.shape) > 0:
                        confidence = box.conf[0].item()  # Get confidence value
                    else:
                        confidence = box.conf.item()  # If it's 0-dimensional, just get the scalar value

                    # Unpack the bounding box (x_min, y_min, x_max, y_max)
                    x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()  # Ensure the tensor is moved to CPU

                    # Calculate the area of the bounding box (width * height)
                    width = x_max - x_min
                    height = y_max - y_min
                    area = width * height

                    # Get the class label (assuming you have the class names from your YOLO model)
                    class_id = int(box.cls[0].item())
                    class_name = self.model.names[class_id]
                    if class_name not in ('erc_over','erc_under'):
                        continue

                    # Print the class name, area, and confidence to the terminal
                    print(f"Class: {class_name}, Area: {area:.2f} pixels, Confidence: {confidence:.4f}")

                    # Plot the detections on the image
                    annotated_frame = result.plot()

            # Display the video feed (with annotations if any)
            cv2.imshow("Live Video Feed", annotated_frame)

            # Calculate time for frame inference
            frame_end_time = time.perf_counter()
            self.frame_times.append(frame_end_time - frame_start_time)

            # Periodically print metrics every 100 frames
            self.print_performance_metrics()

            # Check for key press to continue the video feed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Press 'q' to exit
                self.shutdown_flag = True

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def print_performance_metrics(self):
        total_time = time.time() - self.start_time
        average_inference_time = np.mean(self.frame_times)
        fps = 1 / average_inference_time if average_inference_time > 0 else 0

        print(f"Frames processed: {self.frame_count}")
        print(f"Total video processing time: {total_time:.2f} seconds")
        print(f"Average inference time per frame: {average_inference_time:.4f} seconds")
        print(f"Estimated FPS: {fps:.2f}")

    def on_shutdown(self):
        # Print final performance metrics
        total_time = time.time() - self.start_time
        average_inference_time = np.mean(self.frame_times)
        fps = 1 / average_inference_time if average_inference_time > 0 else 0

        print(f"\nFinal Results after Video Processing:")
        print(f"Total video processing time: {total_time:.2f} seconds")
        print(f"Average inference time per frame: {average_inference_time:.4f} seconds")
        print(f"Estimated FPS: {fps:.2f}")

    def spin_until_shutdown(self):
        """Spin the node until the shutdown flag is set."""
        while not self.shutdown_flag:
            rclpy.spin_once(self)
        self.on_shutdown()


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()

    # Spin the node with a custom loop
    camera_subscriber.spin_until_shutdown()

    # Cleanup
    camera_subscriber.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
