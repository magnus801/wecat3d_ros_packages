#!/usr/bin/env python3
import cv2
import time
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# ────── CONFIG ────────────────────────────────────────────────────────────
MODEL_PATH       = "/home/strix-0/Yolo_models/best_under_over_fir.pt"
CONF_THRESHOLD   = 0.95
IGNORE_CLASS     = "erc_fit"        # ← class to suppress entirely
SNAP_DIR         = Path("/home/strix-0/Yolo_models/erc_snaps")
TOLERANCE_PX     = 100
COOLDOWN_FRAMES  = 30
CENTRE_OFFSET_XY = (-150, 0)
SNAP_SIZE        = (1280, 960)      # output resolution (w, h)
SNAP_DIR.mkdir(parents=True, exist_ok=True)
# ──────────────────────────────────────────────────────────────────────────

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")

        # ROS I/O -----------------------------------------------------------
        self.subscription = self.create_subscription(
            Image,
            "/dart_camera_front_right/pylon_ros2_camera_node_right/image/image_raw",
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Image, "/dart_camera_front_right/annotated", 10)

        # Helpers -----------------------------------------------------------
        self.bridge = CvBridge()
        self.model  = YOLO(MODEL_PATH)
        self.get_logger().info("Camera Subscriber Node Initialised!")

        # Runtime state -----------------------------------------------------
        self.frame_center = None
        self.cooldown     = 0
        self.frame_times  = []
        self.start_time   = time.time()
        self.frame_count  = 0
        self.shutdown_flag = False

    # ---------------------------------------------------------------------
    def listener_callback(self, msg: Image):
        try:
            t0 = time.perf_counter()
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Establish frame centre once
            if self.frame_center is None:
                h, w = frame.shape[:2]
                dx, dy = CENTRE_OFFSET_XY
                self.frame_center = (w // 2 + dx, h // 2 + dy)
                self.get_logger().info(f"Frame centre set to {self.frame_center}")

            # ── YOLO inference ────────────────────────────────────────────
            results = self.model(frame, conf=CONF_THRESHOLD)
            annotated = frame.copy()

            for r in results:
                boxes = r.boxes
                if not boxes:
                    continue

                for b in boxes:
                    conf = float(b.conf.flatten()[0])
                    cls_id = int(b.cls[0])
                    cls_name = self.model.names[cls_id]

                    # Skip the ignored class entirely
                    if cls_name == IGNORE_CLASS:
                        continue

                    x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()
                    area = (x2 - x1) * (y2 - y1)
                    print(f"Class: {cls_name}, Area: {area:.1f}px², Conf: {conf:.4f}")

                    # Draw the box & label
                    self.draw_box(annotated, (x1, y1, x2, y2), cls_name, conf)

                    # Snapshot trigger -----------------------------------
                    if self.cooldown == 0:
                        bx, by = (x1 + x2) / 2, (y1 + y2) / 2
                        dist = np.hypot(bx - self.frame_center[0],
                                        by - self.frame_center[1])
                        if dist < TOLERANCE_PX:
                            self.take_snapshot(frame, (int(x1), int(y1), int(x2), int(y2)))
                            self.cooldown = COOLDOWN_FRAMES

                if self.cooldown:   # no need to process further boxes this frame
                    break

            # Draw green centre box for reference
            cx, cy = self.frame_center
            s = 50
            cv2.rectangle(annotated, (cx - s//2, cy - s//2),
                          (cx + s//2, cy + s//2), (0, 255, 0), 2)

            # Preview & publish
            cv2.imshow("Live Video Feed", annotated)
            out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header.stamp    = self.get_clock().now().to_msg()
            out.header.frame_id = msg.header.frame_id
            self.publisher.publish(out)

            # Book‑keeping
            self.frame_times.append(time.perf_counter() - t0)
            self.frame_count += 1
            self.cooldown = max(0, self.cooldown - 1)
            if self.frame_count % 100 == 0:
                self.print_performance_metrics()

            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                self.shutdown_flag = True

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    # ---------------------------------------------------------------------
    @staticmethod
    def draw_box(img, box, label, conf):
        """Draw a bounding box and text on the image."""
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 165, 255), 2)
        txt = f"{label} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 165, 255), -1)
        cv2.putText(img, txt, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)

    # ---------------------------------------------------------------------
    def take_snapshot(self, frame, box):
        x1, y1, x2, y2 = box
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        crop = frame[y1:y2, x1:x2]
        crop_resized = cv2.resize(crop, SNAP_SIZE, interpolation=cv2.INTER_LINEAR)

        fname = SNAP_DIR / f"crop_{time.strftime('%Y%m%d_%H%M%S')}.png"
        cv2.imwrite(str(fname), crop_resized)
        self.get_logger().info(f"Snapshot saved: {fname}")

    # ---------------------------------------------------------------------
    def print_performance_metrics(self):
        avg_inf = np.mean(self.frame_times) if self.frame_times else 0
        fps = 1 / avg_inf if avg_inf else 0
        elapsed = time.time() - self.start_time
        print(f"\nFrames: {self.frame_count}"
              f"\nElapsed: {elapsed:.1f}s"
              f"\nAvg/frame: {avg_inf:.4f}s"
              f"\nFPS: {fps:.2f}")

    def spin_until_shutdown(self):
        while not self.shutdown_flag:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.print_performance_metrics()

# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        node.spin_until_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
