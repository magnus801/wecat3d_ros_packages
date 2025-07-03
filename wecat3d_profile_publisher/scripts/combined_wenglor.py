#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import numpy as np

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')
        
        self.offset_x = 2.0 

        self.sub1 = self.create_subscription(
            PointCloud2, '/wenglor1/pointcloud', self.callback_pc1, 10)
        self.sub2 = self.create_subscription(
            PointCloud2, '/wenglor2/pointcloud', self.callback_pc2, 10)
        
        self.pub = self.create_publisher(PointCloud2, '/wecat3d/combined', 10)

        self.pc1 = None
        self.pc2 = None

    def callback_pc1(self, msg):
        self.pc1 = msg
        self.publish_combined()

    def callback_pc2(self, msg):
        self.pc2 = msg
        self.publish_combined()

    def publish_combined(self):
    if self.pc1 is None or self.pc2 is None:
        return

    # Extract xyz points from PointCloud2 messages
    arr1 = np.asarray(list(pc2.read_points(self.pc1, field_names=("x", "y", "z"), skip_nans=True)))
    arr2 = np.asarray(list(pc2.read_points(self.pc2, field_names=("x", "y", "z"), skip_nans=True)))

    # Convert structured arrays to float32 (N, 3) format
    points1 = np.vstack([arr1['x'], arr1['y'], arr1['z']]).T.astype(np.float32)
    points2 = np.vstack([arr2['x'], arr2['y'], arr2['z']]).T.astype(np.float32)

    if points1.size == 0 or points2.size == 0:
        self.get_logger().warn("One or both point-clouds are empty.")
        return

    # Apply 180° rotation around the Z-axis: flip X and Y coordinates
    points2[:, 0:2] *= -1  # Flip x and y only (180° rotation around Z-axis)
    points2[:, 0] += self.offset_x  # Translate points2 along X-axis

    # Compute centroids of both point clouds
    centroid1 = np.mean(points1, axis=0)
    centroid2 = np.mean(points2, axis=0)

    # Create line points to interpolate between centroids
    num_line_pts = 200  # increase for smoother line
    line_points = np.linspace(centroid1, centroid2, num=num_line_pts)

    # Merge all points into one combined cloud
    combined_points = np.vstack((points1, points2, line_points))

    # Create and publish the combined PointCloud2 message
    merged_msg = pc2.create_cloud_xyz32(self.pc1.header, combined_points.tolist())
    merged_msg.header.stamp = self.get_clock().now().to_msg()
    self.pub.publish(merged_msg)

    self.get_logger().info(f"Published merged cloud with {len(combined_points)} points (including line).")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()