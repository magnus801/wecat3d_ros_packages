#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime
from pathlib import Path


class ImageDisplayAndRecord(Node):
    def __init__(self):
        super().__init__("image_display_and_record")

        # -------- parameters ----------
        default_topic = "/dart_camera_front_right/pylon_ros2_camera_node_right/image/image_raw"
        self.declare_parameter("image_topic", default_topic)
        image_topic = self.get_parameter("image_topic").value

        self.declare_parameter("fps", 30.0)              # recording FPS fallback
        self.fps = float(self.get_parameter("fps").value)
        # --------------------------------

        self.bridge = CvBridge() 
        self.sub = self.create_subscription(Image, image_topic, self.cb, 10)

        self.get_logger().info(f"Subscribed to {image_topic}")

        # VideoWriter will be opened lazily on the first frame (we need width/height)
        self.writer = None
        self.video_path = None

    # -------------------------------------------------------------
    def cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        # Lazily initialise the VideoWriter once we know frame size
        if self.writer is None:
            height, width = frame.shape[:2]
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.video_path = Path.cwd() / f"camera_record_{ts}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")      # widely supported codec
            self.writer = cv2.VideoWriter(
                str(self.video_path), fourcc, self.fps, (width, height)
            )
            if not self.writer.isOpened():
                self.get_logger().error("Could not open VideoWriter – recording disabled")
                self.writer = None
            else:
                self.get_logger().info(f"Recording to {self.video_path}")

        # write the frame if recording is active
        if self.writer is not None:
            self.writer.write(frame)

        # display
        cv2.imshow("Front-Right Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Quit requested with 'q'")
            rclpy.get_global_executor().stop()

    # -------------------------------------------------------------
    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"Saved video → {self.video_path}")
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ImageDisplayAndRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
