#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import signal
import os
from std_msgs.msg import Header

MIN_X = -0.039
MAX_X = 0.000

stop_requested = False


def get_xyz_points(cloud_msg):
    points = []
    step = cloud_msg.point_step
    fields = {f.name: (f.offset, f.datatype) for f in cloud_msg.fields}
    for i in range(0, len(cloud_msg.data), step):
        x = struct.unpack_from('<f', cloud_msg.data, i + fields['x'][0])[0]
        y = struct.unpack_from('<f', cloud_msg.data, i + fields['y'][0])[0]
        z = struct.unpack_from('<f', cloud_msg.data, i + fields['z'][0])[0]
        points.append((x, y, z))
    return points


def create_pointcloud2(points, frame_id, stamp):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    point_step = 12  # 3 floats * 4 bytes
    data = b''.join(struct.pack('<fff', *pt) for pt in points)

    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * len(points),
        data=data,
        is_dense=True
    )

class SimplePointCloudLogger(Node):
    def __init__(self, file_path):
        super().__init__('simple_pointcloud_logger')
        self.file = open(file_path, 'a')
        self.counter = 0
        self.subscription = self.create_subscription(
            PointCloud2,
            '/wecat3d/pointcloud',
            self.pointcloud_callback,
            10
        )
        self.cloud_pub = self.create_publisher(PointCloud2, '/wecat3d/gauge_cloud', 10)

    def pointcloud_callback(self, msg):
        points = get_xyz_points(msg)
        filtered_points = [(x, y, z) for x, y, z in points if MIN_X <= x <= MAX_X]

        for _, _, z in filtered_points:
            self.file.write(f"{z:.6f}\n")

        self.file.flush()
        self.counter += 1

        cloud_out = create_pointcloud2(
            points=filtered_points,
            frame_id="map",
            stamp=msg.header.stamp
        )
        self.cloud_pub.publish(cloud_out)

    def stop(self):
        self.file.close()
        self.destroy_node()


def signal_handler(sig, frame):
    global stop_requested
    stop_requested = True
    print("Stopping the logger...")


def main():
    global stop_requested
    rclpy.init()

    output_dir = os.getenv("PCD_OUTPUT_DIR", "/tmp")
    os.makedirs(output_dir, exist_ok=True)
    file_path = os.path.join(output_dir, "pointcloud_log.txt")

    logger_node = SimplePointCloudLogger(file_path)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(logger_node, timeout_sec=0.1)
    finally:
        logger_node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
