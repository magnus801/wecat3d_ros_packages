#!/usr/bin/env python3

# ──────────────────────────────────────────────────────────────────────
import cv2, time, json, datetime, numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
# ──────────────────────────────────────────────────────────────────────
# CONFIGURATION

MODEL_PATH       = "/home/strix-0/Yolo_models/final.pt"
CONF_THRESHOLD   = 0.97

SNAP_DIR         = Path("/home/strix-0/Yolo_models/erc_snaps")
LOG_FILE         = SNAP_DIR / "erc_log.jsonl"

SNAP_SIZE        = (640,640)      # saved crop size (w, h)
TOLERANCE_PX     = 120              # centre‑box tolerance
COOLDOWN_FRAMES  = 30               # frames to skip after a snap
CENTRE_OFFSET_XY = (-150, 0)        # px offset of trigger centre
# ──────────────────────────────────────────────────────────────────────

SNAP_DIR.mkdir(parents=True, exist_ok=True)


class CameraWithLogging(Node):
    # ──────────────────────────────
    # INITIALISATION
    # ──────────────────────────────
    def __init__(self):
        super().__init__("camera_with_logging")

        # image subscription
        self.create_subscription(
            Image,
            "/dart_camera_front_right/pylon_ros2_camera_node_right/image/image_raw",
            self.image_cb,
            10)

        # annotated image publisher
        self.publisher = self.create_publisher(
            Image,
            "/dart_camera_front_right/annotated",
            10)

        # distance subscription (from CloudToPose)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "pose_cov",
            self.pose_cb,
            10)

        # helpers / state
        self.bridge      = CvBridge()
        self.model       = YOLO(MODEL_PATH)

        self.frame_center = None             # trigger centre (px)
        self.cooldown     = 0
        self.frame_times  = []
        self.frame_count  = 0
        self.start_time   = time.time()
        self.shutdown     = False

        self._y0_mm       = None             # baseline
        self.y_mm         = None             # latest relative distance
        self.erc_status   = "fit"            # current clip state
        self.frame_seq    = 0                # running counter

        self.log_fh = LOG_FILE.open("a", buffering=1)  # line‑buffered

        self.get_logger().info("Camera‑with‑logging node initialised")

    # ──────────────────────────────
    # DISTANCE CALLBACK
    # ──────────────────────────────
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """Update relative distance and write a log line."""
        raw_mm = msg.pose.pose.position.x  # mm

        # initialise baseline on first reading
        # if self._y0_mm is None:
        #     self._y0_mm = raw_mm

        # self.y_mm = raw_mm - self._y0_mm
        self.y_mm = raw_mm 

        # continuous pose log (no image)
        self.log_event()

    # ──────────────────────────────
    # IMAGE CALLBACK
    # ──────────────────────────────
    def image_cb(self, msg: Image):
        try:
            tic = time.perf_counter()

            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # one‑time centre calculation
            if self.frame_center is None:
                h, w = frame.shape[:2]
                dx, dy = CENTRE_OFFSET_XY
                self.frame_center = (w // 2 + dx, h // 2 + dy)
                self.get_logger().info(f"Trigger centre set to {self.frame_center}")

            # run YOLO
            results   = self.model(frame, conf=CONF_THRESHOLD)
            annotated = frame.copy()

            for r in results:
                if not (r.boxes and len(r.boxes)):
                    continue
                annotated = r.plot()

                for b in r.boxes:
                    cls_name = self.model.names[int(b.cls[0])]
                    if cls_name == "erc_fit":           # skip this class
                        continue

                    x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()
                    bx, by = (x1 + x2) / 2, (y1 + y2) / 2

                    if (self.cooldown == 0 and
                        np.hypot(bx - self.frame_center[0],
                                 by - self.frame_center[1]) < TOLERANCE_PX):

                        # change global status, take snapshot, reset cooldown
                        self.erc_status = cls_name       # over / under
                        self.take_snapshot(
                            frame,
                            (int(x1), int(y1), int(x2), int(y2)),
                            cls_name)
                        self.cooldown = COOLDOWN_FRAMES
                        break          # only one snapshot per frame

            # draw trigger box for preview
            cx, cy = self.frame_center
            cv2.rectangle(annotated, (cx-25, cy-25), (cx+25, cy+25),
                          (0, 255, 0), 2)

            # show preview window (Esc/‘q’ to quit)
            cv2.imshow("Live", annotated)

            # publish annotated image
            ros_img = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            ros_img.header.stamp    = msg.header.stamp
            ros_img.header.frame_id = msg.header.frame_id
            self.publisher.publish(ros_img)

            # performance bookkeeping
            self.frame_times.append(time.perf_counter() - tic)
            self.frame_count += 1
            self.cooldown = max(0, self.cooldown - 1)
            if self.frame_count % 100 == 0:
                self.print_metrics()

            # keyboard quit
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                self.shutdown = True

        except Exception as e:
            self.get_logger().error(f"image_cb error: {e}")

    # ──────────────────────────────
    # SNAPSHOT + EVENT LOGGING
    # ──────────────────────────────
    def take_snapshot(self, frame, box_px, cls_name):
        """Save PNG and log a snapshot record."""
        x1, y1, x2, y2 = box_px
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        crop = frame[y1:y2, x1:x2]
        crop = cv2.resize(crop, SNAP_SIZE)

        ts = time.strftime('%Y%m%d_%H%M%S')
        fname = SNAP_DIR / f"{ts}_{cls_name}.png"
        cv2.imwrite(str(fname), crop)
        self.get_logger().info(f"Snapshot saved: {fname}")

        # log with snapshot field
        self.log_event(image_name=fname.name, snapshot=True)
        self.erc_status = "fit"
    # ------------------------------------------------------------------
        # ------------------------------------------------------------------
    def log_event(self, *, image_name=None, snapshot=False):
        """Append one JSON line (pose update or snapshot)."""
        now = time.time()
        ts12h = datetime.datetime.fromtimestamp(now).strftime("%I:%M:%S %p")

        # ── NEW: mark bad states as “not ok” ───────────────────────────
        if self.erc_status in ("erc_over", "erc_under"):
            status_flag = "not ok"
        else:
            status_flag = "ok"
        # ───────────────────────────────────────────────────────────────

        record = {
            "date":       datetime.date.fromtimestamp(now).isoformat(),
            "frame_seq":  self.frame_seq,
            "log_title":  "Rail Head Profiling",
            "y_mm":       self.y_mm,
            "erc_status": self.erc_status,
            "status":     status_flag,          # ← uses the new flag
            "timestamp":  ts12h
        }
        if snapshot and image_name:
            record["snapshot"] = image_name

        json.dump(record, self.log_fh)
        self.log_fh.write("\n")
        self.frame_seq += 1

    # ──────────────────────────────
    # METRICS + SHUTDOWN
    # ──────────────────────────────
    def print_metrics(self):
        avg = np.mean(self.frame_times)
        fps = 1 / avg if avg else 0
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f"\nFrames:   {self.frame_count}"
            f"\nElapsed:  {elapsed:.1f}s"
            f"\nAvg/frame:{avg:.4f}s"
            f"\nFPS:      {fps:.2f}")

    def spin_until_shutdown(self):
        while not self.shutdown:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.print_metrics()

    def destroy_node(self):
        super().destroy_node()
        self.log_fh.close()
        self.get_logger().info("Node shut down, log closed.")


# ──────────────────────────────
# MAIN
# ──────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = CameraWithLogging()
    try:
        node.spin_until_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
