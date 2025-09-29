#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import struct

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
        # ► single publisher now – only for point clouds
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/wenglor/combined_pointcloud', 10)
        self.get_logger().info('Wenglor merger ▶ /wenglor/combined_pointcloud')
    
    def rgb_to_float(self, r, g, b):
        """Convert RGB values (0-255) to a float32 for point cloud color field."""
        rgb_int = (r << 16) | (g << 8) | b
        return struct.unpack('f', struct.pack('I', rgb_int))[0]
    
    def create_colored_cloud(self, points, colors, header):
        """Create a colored point cloud from points and colors arrays."""
        # Create the fields for XYZRGB
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create structured array with XYZRGB data
        cloud_data = []
        for i in range(len(points)):
            cloud_data.append([
                points[i][0],  # x
                points[i][1],  # y
                points[i][2],  # z
                colors[i]      # rgb as float
            ])
        
        # Create the point cloud messagewecat3d_ros_packages
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(cloud_data)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 bytes each for x,y,z,rgb
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        # Pack the data
        cloud_data_bytes = b''
        for point in cloud_data:
            cloud_data_bytes += struct.pack('ffff', point[0], point[1], point[2], point[3])
        
        cloud_msg.data = cloud_data_bytes
        return cloud_msg

    # ------------------------------------------------------------------ #
    def merge_cb(self, cloud_left: PointCloud2, cloud_right: PointCloud2):
        """Mirror right cloud in X, translate by sensor_offset, merge & add centroid line as extra points."""
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
        
        theta_L = np.deg2rad(-48.0)
        cL, sL  = np.cos(theta_L), np.sin(theta_L)
        x, z = pts_left[:, 0].copy(), pts_left[:, 2].copy()
        pts_left[:, 0] =  cL * x +  sL * z          # X′
        pts_left[:, 2] = -sL * x +  cL * z          # Z′
        
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
        
        # ► generate "line" points between the two centroids
        num_line_pts = 200                           # increase for smoother line
        line_points = np.linspace(c_left, c_right, num=num_line_pts)
        
        # --- Create colors for each point cloud section -----------------
        # Define colors (RGB values 0-255)
        left_color = self.rgb_to_float(0, 0, 255)    # Red for left sensor
        right_color = self.rgb_to_float(0, 0, 255)   # Green for right sensor  
        line_color = self.rgb_to_float(255, 0, 0)    # Blue for connecting line
        
        # Create color arrays for each section
        left_colors = np.full(len(pts_left), left_color)
        right_colors = np.full(len(pts_right), right_color)
        line_colors = np.full(len(line_points), line_color)
        
        # --- merge original clouds *plus* line points -------------------
        merged_pts = np.vstack((pts_left, pts_right, line_points))
        merged_colors = np.concatenate((left_colors, right_colors, line_colors))
        
        # ► publish combined colored cloud
        hdr = cloud_left.header
        hdr.stamp = self.get_clock().now().to_msg()
        
        colored_cloud = self.create_colored_cloud(merged_pts, merged_colors, hdr)
        self.cloud_pub.publish(colored_cloud)

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