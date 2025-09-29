#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point
from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import Marker

class WenglorMerger(Node):
    def __init__(self):
        super().__init__('wenglor_merger')

        # Distance between the two sensors (metres)
        self.declare_parameter('sensor_offset', 4.0)

        # Subscribe with time synchroniser
        self.sub_left  = Subscriber(self, PointCloud2, '/wenglor1/pointcloud')
        self.sub_right = Subscriber(self, PointCloud2, '/wenglor2/pointcloud')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.merge_cb)

        # Publishers
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/wenglor/combined_pointcloud', 10)
        self.line_pub = self.create_publisher(
            Marker, '/wenglor/centroid_line', 10)

        self.get_logger().info('Wenglor merger ▶ /wenglor/combined_pointcloud')

    # ------------------------------------------------------------------ #
    def merge_cb(self, cloud_left: PointCloud2, cloud_right: PointCloud2):
        """Mirror right cloud in X, translate by sensor_offset, merge & draw line."""
        offset = self.get_parameter('sensor_offset').get_parameter_value().double_value

        # --- convert clouds to (N,3) float32 matrices -------------------
        pts_left  = pc2.read_points_numpy(
            cloud_left,  field_names=('x', 'y', 'z'), skip_nans=True
        ).view(np.float32).reshape(-1, 3)

        pts_right = pc2.read_points_numpy(
            cloud_right, field_names=('x', 'y', 'z'), skip_nans=True
        ).view(np.float32).reshape(-1, 3)

        if pts_left.size == 0 or pts_right.size == 0:
            self.get_logger().warn('Empty cloud – skipping frame')
            return

        # --- mirror & shift right sensor --------------------------------
        pts_left[:, [0, 2]]  *= -1          # flip X and Z
        pts_right[:, [0, 2]] *= -1

        theta_L = np.deg2rad(-45.0)
        cL, sL  = np.cos(theta_L), np.sin(theta_L)
        x, z = pts_left[:, 0].copy(), pts_left[:, 2].copy()
        pts_left[:, 0] =  cL * x +  sL * z          # X′
        pts_left[:, 2] = -sL * x +  cL * z          # Z′

        #    right sensor +45 °     (counter-clockwise)
        theta_R = np.deg2rad(-45.0)
        cR, sR  = np.cos(theta_R), np.sin(theta_R)
        x, z = pts_right[:, 0].copy(), pts_right[:, 2].copy()
        pts_right[:, 0] =  cR * x +  sR * z
        pts_right[:, 2] = -sR * x +  cR * z
        pts_right[:, 0] *= -1        # mirror X
        pts_right[:, 0] += offset    # shift by wheel spacing

        # --- centroids BEFORE merge -------------------------------------
        c_left  = pts_left.mean(axis=0)
        c_right = pts_right.mean(axis=0)

        # --- merge & publish cloud --------------------------------------
        merged_pts = np.vstack((pts_left, pts_right))
        hdr = cloud_left.header
        hdr.stamp = self.get_clock().now().to_msg()
        merged_cloud = pc2.create_cloud_xyz32(hdr, merged_pts)
        self.cloud_pub.publish(merged_cloud)

        # --- publish line marker ----------------------------------------
        line = Marker()
        line.header = hdr                   # same frame ('map')
        line.ns = "wenglor"; line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.01                 # width in metres
        line.color.r = 1.0; line.color.a = 1.0

        line.points = [
            Point(x=float(c_left[0]),  y=float(c_left[1]),  z=float(c_left[2])),
            Point(x=float(c_right[0]), y=float(c_right[1]), z=float(c_right[2]))
        ]

        self.line_pub.publish(line)

# --------------------------------------------------------------------- #
def main():
    rclpy.init()
    node = WenglorMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
