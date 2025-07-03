#!/usr/bin/env python3
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


class CloudToPose(Node):
    def __init__(self):
        super().__init__("cloud_to_pose")

        # ---------- parameters ----------
        self.declare_parameter("odom_frame",  "odom")
        self.declare_parameter("base_frame",  "base_link")
        self.declare_parameter("publish_tf",  True)

        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        # ---------- publisher ----------
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              "pose_cov", 10)

        # ---------- subscriber ----------
        self.create_subscription(PointCloud2,
                                 "/wenglor1/pointcloud",
                                 self.cloud_cb,
                                 qos_profile_sensor_data)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._y_offset = None            # filled on first message
        self._point_step = None

        self.get_logger().info("cloud_to_pose node ready.")

    def cloud_cb(self, cloud: PointCloud2):
        if self._y_offset is None:
            foffsets = {f.name: f.offset for f in cloud.fields}
            if "y" not in foffsets:
                self.get_logger().error("No 'y' field in cloud!")
                return
            self._y_offset   = foffsets["y"]
            self._point_step = cloud.point_step

        if len(cloud.data) < self._point_step:
            return  

        y_m = struct.unpack_from('<f', cloud.data, self._y_offset)[0]

        stamp = self.get_clock().now().to_msg()

        # ---------- PoseWithCovarianceStamped -----------------------------
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.pose.position.x = y_m      
        pose_msg.pose.pose.orientation.w = 1.0      

        cov = [0.0]*36
        cov[0]  = 1e-4      
        cov[7]  = 1.0       
        cov[14] = 1.0       
        cov[21] = 3.14      
        cov[28] = 3.14      
        cov[35] = 3.14      
        pose_msg.pose.covariance = cov

        self.pose_pub.publish(pose_msg)

        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header        = pose_msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = y_m
            tf.transform.rotation.w    = 1.0
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToPose()
    try:
        rclpy.spin(node)      
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
