#!/usr/bin/env python3
"""
velocity_kmh_filtered.py
Report running-average and filtered-instantaneous speed in km/h
from y-position (millimetres) on the pose_cov topic.
"""

from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

KMH_PER_MM_S = 0.0036   # 1 mm/s = 0.0036 km/h
WINDOW = 5              # moving-average length for the instantaneous speed


class VelocityKmHFiltered(Node):
    def __init__(self):
        super().__init__("velocity_kmh_filtered")

        # -------- parameters --------
        self.declare_parameter("pose_topic", "pose_cov")
        topic = self.get_parameter("pose_topic").get_parameter_value().string_value

        # -------- state --------
        self.start_time = None   # seconds (first sample)
        self.start_pos = None    # millimetres
        self.prev_time = None    # previous sample time
        self.prev_pos = None     # previous y position
        self.window = deque(maxlen=WINDOW)

        # -------- subscription --------
        self.create_subscription(
            PoseWithCovarianceStamped,
            topic,
            self.pose_cb,
            10
        )

        self.get_logger().info(
            f"Listening to “{topic}”.  Press Ctrl-C to stop."
        )

    # ------------------------------------------------------------------
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """Runs on every incoming PoseWithCovarianceStamped."""
        y_mm = msg.pose.pose.position.x
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # seconds

        # first sample --------------------------------------------------
        if self.start_time is None:
            self.start_time = t
            self.start_pos = y_mm
            self.prev_time = t
            self.prev_pos = y_mm
            self.get_logger().info("First pose received – timer started.")
            return

        # running-average velocity -------------------------------------
        dt_run = t - self.start_time
        dy_run = y_mm - self.start_pos
        v_avg_mm_s = dy_run / dt_run if dt_run > 0.0 else float("nan")
        v_avg_kmh = v_avg_mm_s * KMH_PER_MM_S

        # instantaneous velocity ---------------------------------------
        dt_inst = t - self.prev_time
        dy_inst = y_mm - self.prev_pos
        v_inst_mm_s = dy_inst / dt_inst if dt_inst > 0.0 else float("nan")

        # update moving-average filter (ignore NaN)
        if v_inst_mm_s == v_inst_mm_s:
            self.window.append(v_inst_mm_s)
        v_inst_filt_mm_s = sum(self.window) / len(self.window)
        v_inst_filt_kmh = v_inst_filt_mm_s * KMH_PER_MM_S

        # log results ---------------------------------------------------
        self.get_logger().info(
            f"t={dt_run:5.2f} s  "
            f"pos={y_mm:9.2f} mm  "
            f"v_avg={v_avg_kmh:6.2f} km/h  "
            f"v_now={v_inst_filt_kmh:6.2f} km/h"
        )

        # store current sample as previous for next iteration
        self.prev_time = t
        self.prev_pos = y_mm


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = VelocityKmHFiltered()

    try:
        rclpy.spin(node)            # Ctrl-C -> KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
