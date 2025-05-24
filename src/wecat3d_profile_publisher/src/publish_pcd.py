import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')

        self.publisher_ = self.create_publisher(PointCloud2, 'merged_pcd', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pcd = self.load_pcd("merged.pcd")

    def load_pcd(self, filename):
        pcd = o3d.io.read_point_cloud(filename)
        return np.asarray(pcd.points)

    def timer_callback(self):
        msg = self.convert_cloud_to_ros_msg(self.pcd)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing merged_output_keyboard.pcd')

    def convert_cloud_to_ros_msg(self, cloud_array):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"  

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for x, y, z in cloud_array:
            cloud_data.append(struct.pack('fff', x, y, z))
        cloud_data = b"".join(cloud_data)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(cloud_array)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = cloud_data

        return msg

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

