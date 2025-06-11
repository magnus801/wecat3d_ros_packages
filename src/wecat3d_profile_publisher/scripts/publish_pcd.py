#!/usr/bin/env python3
import os
import platform
import time
import logging
from dataclasses import dataclass
from ctypes import *
import numpy as np
import struct

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import threading

logging.basicConfig(level=logging.DEBUG)
SENSOR_DISCONNECTED = 0
SENSOR_CONNECTED = 3
ENC_SCALE_MM = 0.02
PCD_FILE = "merged.pcd"
SENSOR_BUFFERSIZEMAX = 4202500
Sensor_OK = 0
Sensor_ERROR = -1
Sensor_GETINFOSMALLBUFFER = -2
Sensor_GETINFONOVALIDINFO = -3
Sensor_GETINFOINVALIDXML = -4
Sensor_GETXZINONEWSCAN = -1
Sensor_GETXZIINVALIDLINDATA = -2
Sensor_GETXZIINVALIDBUFFER = -3
Sensor_READDATAOK = 0
Sensor_READDATASMALLBUFFER = -1
Sensor_READDATANOTSUPPORTEDMODE = -2
Sensor_READDATAFEATURENOTDEFINED = -3
Sensor_READDATANOSCAN = -4
Sensor_READDATAFAILED = -5
Sensor_WRITEDATAINVALIDSOCKET = -1
Sensor_INVALIDHANDLE = -1000
Sensor_GETINFOSMALLERBUFFER = -2

class SensorException(Exception):
    def __init__(self, errcode):
        super().__init__(f'Failed: Sensor (DLL) returned error code: {errcode}. Please check the DLL documentation for the error code.')

# region data classes
@dataclass
class UserIOState:
    EA1: int
    EA2: int
    EA3: int
    EA4: int
    TTLEncA: int
    TTLEncB: int
    TTLEncC: int

# Contains the data of a single scan
@dataclass
class ScannedProfile:
    # Large array of floating point coordinate X
    roiWidthX: np.ndarray
    # Large array of floating point coordinate Z
    roiHeightZ: np.ndarray
    # 8-bit monochrome intensity
    intensity: np.ndarray
    # Peak width (aka Signal Width)
    signalWidth: np.ndarray
    # Encoder value
    encoderValue: np.uintc
    # User IO State
    userIOState: UserIOState
    # Picture counter value
    pictureCounter: int
    # Scanned scannedPoints:
    scannedPoints: int

@dataclass
class CameraImage:
    rawImageData: np.ndarray
    imgWidth: int
    imgHeight: int
    imgOffsetX: int
    imgOffsetY: int
    imgStepX: int
    imgStepY: int
# endregion

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/sensor/pointcloud', 10)
        self.get_logger().info('Point Cloud Publisher Node has been started.')
    
    def create_pointcloud2_msg(self, points, frame_id='map'):
        """
        Create a PointCloud2 message from a list of [x, y, z] points
        """
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        
        # Define point fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.point_step = 12  # 4 bytes per field * 3 fields
        msg.row_step = msg.point_step * msg.width
        
        # Pack point data
        data = []
        for point in points:
            x, y, z = point
            data.append(struct.pack('fff', float(x), float(y), float(z)))
        
        msg.data = b''.join(data)
        return msg
    
    def publish_pointcloud(self, points):
        """
        Publish a point cloud
        """
        if len(points) > 0:
            msg = self.create_pointcloud2_msg(points)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published point cloud with {len(points)} points')

# lib = cdll.LoadLibrary(os.path.join(os.getcwd(), 'EthernetScanner.dll')) if platform.system() == "Windows" else cdll.LoadLibrary(os.path.join(os.getcwd(), 'libEthernetScanner.so'))
lib_path = "/home/strix-0/wenglor_ws/wecat3d_ros_packages/src/wecat3d_sdk/lib/libEthernetScanner.so"
lib = cdll.LoadLibrary(lib_path)

EthernetScanner_Connect = lib.EthernetScanner_Connect
EthernetScanner_Connect.restype = c_void_p
EthernetScanner_Connect.argtypes = [c_char_p, c_char_p, c_int]

EthernetScanner_Disconnect = lib.EthernetScanner_Disconnect
EthernetScanner_Disconnect.restype = c_void_p
EthernetScanner_Disconnect.argtypes = [c_void_p]

EthernetScanner_GetConnectStatus = lib.EthernetScanner_GetConnectStatus
EthernetScanner_GetConnectStatus.restype = None
EthernetScanner_GetConnectStatus.argtypes = [c_void_p, POINTER(c_int)]

EthernetScanner_GetXZIExtended = lib.EthernetScanner_GetXZIExtended
EthernetScanner_GetXZIExtended.restype = c_int
EthernetScanner_GetXZIExtended.argtypes = [c_void_p,
                                           POINTER(c_double), POINTER(c_double), POINTER(c_int), POINTER(c_int),
                                           c_int, POINTER(c_uint), POINTER(c_uint), c_int, c_char_p, c_int,
                                           POINTER(c_int)]

EthernetScanner_GetImage = lib.EthernetScanner_GetImage
EthernetScanner_GetImage.restype = c_int
EthernetScanner_GetImage.argtypes = [c_void_p, c_char_p, c_int, POINTER(c_uint), POINTER(c_uint), POINTER(c_uint), POINTER(c_uint), POINTER(c_uint), POINTER(c_uint), c_uint]

EthernetScanner_GetDllFiFoState = lib.EthernetScanner_GetDllFiFoState
EthernetScanner_GetDllFiFoState.restype = c_int
EthernetScanner_GetDllFiFoState.argtypes = [c_void_p]

EthernetScanner_ResetDllFiFo = lib.EthernetScanner_ResetDllFiFo
EthernetScanner_ResetDllFiFo.restype = c_int
EthernetScanner_ResetDllFiFo.argtypes = [c_void_p]

EthernetScanner_GetVersion = lib.EthernetScanner_GetVersion
EthernetScanner_GetVersion.restype = c_int
EthernetScanner_GetVersion.argtypes = [c_char_p, c_int]

EthernetScanner_WriteData = lib.EthernetScanner_WriteData
EthernetScanner_WriteData.restype = c_int
EthernetScanner_WriteData.argtypes = [c_void_p, c_char_p, c_int]

EthernetScanner_ReadData = lib.EthernetScanner_ReadData
EthernetScanner_ReadData.restype = c_int
EthernetScanner_ReadData.argtypes = [c_void_p, c_char_p, c_char_p, c_int, c_int]

class Sensor:
    def __init__(self, ip, port):
        self.ip = ip.encode()
        self.port = str(port).encode()
        self._handle = POINTER(c_int)()
        self.read_buf_size = 128 * 1024
        self.read_buf = (c_char * self.read_buf_size)()
        self._status = pointer(c_int(0))
        # Profile buffers
        self.roiWidthX = POINTER(c_double)()
        self.roiHeightZ = POINTER(c_double)()
        self.intensity = POINTER(c_int)()
        self.signalWidth = POINTER(c_int)()
        self.encoderValue = POINTER(c_uint)()
        self.userIOState = POINTER(c_uint)()
        self.pictureCounter = POINTER(c_int)()
        # Camera image buffers
        self.rawBuffer = POINTER(c_char)()
        self.rawBufferSize = 0
        self.width = POINTER(c_uint)()
        self.height = POINTER(c_uint)()
        self.offsetX = POINTER(c_uint)()
        self.offsetY = POINTER(c_uint)()
        self.stepX = POINTER(c_uint)()
        self.stepY = POINTER(c_uint)()

    def connect(self, timeout=0):
        logging.debug(f'Attempting to connect with the sensor at {self.ip.decode("utf-8")} ...')
        self._handle = EthernetScanner_Connect(self.ip, self.port, c_int(timeout))
        time.sleep(0.5)
        connectionPollingStart = time.time()
        while not self._handle and time.time() - connectionPollingStart < timeout / 1000:
            continue
        connectionStatus = self.get_connect_status()
        if connectionStatus != SENSOR_CONNECTED:
            raise SensorException(f'Failed to connect with the sensor; Connection status: {connectionStatus}')
        logging.debug(f'Sensor has been connected successfully. Status: {connectionStatus}')
        self.allocate_memory()

    def disconnect(self):
        self._handle = lib.EthernetScanner_Disconnect(self._handle)
        # time.sleep(0.5)
        if self._handle:
            raise SensorException(f'Failed to disconnect the sensor.')
        logging.debug(f'Sensor has been disconnected successfully. Status: {SENSOR_DISCONNECTED}')
        self.deallocate_memory()

    def get_connect_status(self):
        EthernetScanner_GetConnectStatus(self._handle, self._status)
        return SENSOR_CONNECTED if int(self._status[0]) & SENSOR_CONNECTED else SENSOR_DISCONNECTED

    def allocate_memory(self):
        logging.debug('Allocating memory for profile and camera image ...')
        # Profile buffers
        self.roiWidthX = (c_double * SENSOR_BUFFERSIZEMAX)(0)
        self.roiHeightZ = (c_double * SENSOR_BUFFERSIZEMAX)(0)
        self.intensity = (c_int * SENSOR_BUFFERSIZEMAX)(0)
        self.signalWidth = (c_int * SENSOR_BUFFERSIZEMAX)(0)
        self.encoderValue = pointer(c_uint(0))
        self.userIOState = pointer(c_uint(0))
        self.pictureCounter = pointer(c_int(0))
        # Camera image buffers
        self.rawBuffer = (c_char * SENSOR_BUFFERSIZEMAX)(0)
        self.rawBufferSize = c_int(SENSOR_BUFFERSIZEMAX)
        self.width = (c_uint * SENSOR_BUFFERSIZEMAX)(0)
        self.height = (c_uint * SENSOR_BUFFERSIZEMAX)(0)
        self.offsetX = (c_uint * SENSOR_BUFFERSIZEMAX)(0)
        self.offsetY = (c_uint * SENSOR_BUFFERSIZEMAX)(0)
        self.stepX = (c_uint * SENSOR_BUFFERSIZEMAX)(0)
        self.stepY = (c_uint * SENSOR_BUFFERSIZEMAX)(0)
        logging.debug('Memory has been allocated successfully')

    def deallocate_memory(self):
        # Profile buffers
        self.roiWidthX = POINTER(c_double)()
        self.roiHeightZ = POINTER(c_double)()
        self.intensity = POINTER(c_int)()
        self.signalWidth = POINTER(c_int)()
        self.encoderValue = POINTER(c_uint)()
        self.userIOState = POINTER(c_uint)()
        self.pictureCounter = POINTER(c_int)()
        # Camera image buffers
        self.rawBuffer = POINTER(c_char)()
        self.rawBufferSize = 0
        self.width = POINTER(c_uint)()
        self.height = POINTER(c_uint)()
        self.offsetX = POINTER(c_uint)()
        self.offsetY = POINTER(c_uint)()
        self.stepX = POINTER(c_uint)()
        self.stepY = POINTER(c_uint)()

    def get_camera_image(self, timeout=3000):
        logging.debug("Reading camera image ...")
        response = EthernetScanner_GetImage(self._handle, self.rawBuffer, self.rawBufferSize, self.width, self.height, self.offsetX, self.offsetY, self.stepX, self.stepY, c_uint(timeout))
        if response < 0:
            raise SensorException(response)
        logging.debug(f'Camera image received successfully')
        img_resolution = self.width[0] * self.height[0]
        img_buf = np.ctypeslib.as_array(self.rawBuffer, shape=(img_resolution, 1))
        return CameraImage(
            rawImageData=np.copy(np.frombuffer(img_buf, dtype=np.uint8, count=img_resolution).reshape((self.height[0], self.width[0]))),
            imgWidth=self.width[0],
            imgHeight=self.height[0],
            imgOffsetX=self.offsetX[0],
            imgOffsetY=self.offsetY[0],
            imgStepX=self.stepX[0],
            imgStepY=self.stepY[0]
        )

    def get_scanned_profile(self, timeout=1000):
        logging.debug("Reading scanned profile ...")
        ucBufferRaw = POINTER(c_char)()  # Deprecated parameter hence passed as NULL as per documentation
        iBufferRaw = 0  # POINTER(c_int)()  # Deprecated parameter hence passed as NULL as per documentation
        response = EthernetScanner_GetXZIExtended(
            self._handle,
            self.roiWidthX, self.roiHeightZ, self.intensity, self.signalWidth, self.rawBufferSize,
            self.encoderValue, self.userIOState,
            c_int(timeout), ucBufferRaw, iBufferRaw,
            self.pictureCounter
        )
        if response < 0:
            raise SensorException(response)
        logging.debug(f'Profile received successfully for {response} points')
        userIOState = UserIOState(
            (self.userIOState[0] & 0b1),
            (self.userIOState[0] & 0b10) >> 1,
            (self.userIOState[0] & 0b100) >> 2,
            (self.userIOState[0] & 0b1000) >> 3,
            (self.userIOState[0] & 0b10000) >> 4,
            (self.userIOState[0] & 0b100000) >> 5,
            (self.userIOState[0] & 0b1000000) >> 6
        )
        return ScannedProfile(
            roiWidthX=np.copy(np.ctypeslib.as_array(self.roiWidthX, shape=(response, 1))[:response]),
            roiHeightZ=np.copy(np.ctypeslib.as_array(self.roiHeightZ, shape=(response, 1))[:response]),
            intensity=np.copy(np.ctypeslib.as_array(self.intensity, shape=(response, 1))[:response]),
            signalWidth=np.copy(np.ctypeslib.as_array(self.signalWidth, shape=(response, 1))[:response]),
            encoderValue=self.encoderValue[0],
            userIOState=userIOState,
            pictureCounter=self.pictureCounter[0],
            scannedPoints=response
        )

    def get_dll_version(self):
        response = EthernetScanner_GetVersion(self.read_buf, self.read_buf_size)
        if response < 0:
            raise SensorException(response)
        return self.read_buf.value.decode()

    def read_data(self, command, cacheTime=0):
        command_bytes = command.encode('ascii')
        cmd_raw_buffer = create_string_buffer(command_bytes)
        response = EthernetScanner_ReadData(self._handle, cmd_raw_buffer, self.read_buf, self.read_buf_size, c_int(cacheTime))
        if response == Sensor_READDATAOK:
            logging.debug(f"Sent Read Data command {command_bytes} -> Response: {self.read_buf.value} (Sensor_READDATAOK)")
        else:
            raise SensorException(response)
        return self.read_buf.value

    def write_data(self, command):
        command_bytes = command.encode('ascii')
        cmd_raw_buffer = create_string_buffer(command_bytes)
        response = EthernetScanner_WriteData(self._handle, cmd_raw_buffer, len(command_bytes))
        response_exp = len(command_bytes)
        if response == response_exp:
            logging.debug(f"Sent Write Data command {command_bytes} -> Response: {response} (len(command))")
        elif 0 < response < response_exp:
            logging.warning(f'Sent Write Data command {command_bytes} -> Response: {response}; Expected: {response_exp} (len(command))')
        else:
            raise SensorException(response)
        time.sleep(0.010)
        return response

def ros_spin_thread(node):
    """Function to run ROS spinning in a separate thread"""
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

def main():
    # Initialize ROS2
    rclpy.init()
    pointcloud_publisher = PointCloudPublisher()
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(pointcloud_publisher,))
    ros_thread.daemon = True
    ros_thread.start()
    
    ipaddress = "192.168.100.1"
    sensor = Sensor(ipaddress, 32001)
    print(f'DLL Version: {sensor.get_dll_version()}')
    sensor.connect(timeout=0)
    sensor.write_data("SetHeartbeat=1000")
    print(f"Sensor status: {sensor.get_connect_status()}")
    sensor.write_data("SetAcquisitionStop")
    time.sleep(0.5)
    print(f'Order number: {sensor.read_data("GetOrderNumber")}')
    print(f'Firmware Version: {sensor.read_data("GetFirmwareVersion")}')
    
    # Encoder-triggered config
    sensor.write_data("SetExposureTime=750")
    print(f'Exposure Time: {sensor.read_data("GetExposureTime")}')
    sensor.write_data("SetTriggerSource=2")
    sensor.write_data("SetTriggerEncoderStep=20")
    sensor.write_data("SetEncoderTriggerFunction=2")
    sensor.write_data("ResetEncoder")
    sensor.write_data("SetRangeImageNrProfiles=1")
   
    sensor.write_data("SetAcquisitionStart")
    scanNo = 0
    last_encoder = None
    all_points = []
    
    # Configuration for ROS2 publishing
    INTENSITY_THRESHOLD = 25  # Skip low-intensity points
    PUBLISH_INTERVAL = 10  # Publish every N scans
    
    print("Sensor is running with encoder triggering. Press Ctrl+C to stop.")
    try:
        while True:
            try:
                scan = sensor.get_scanned_profile(timeout=1000)
                scanNo += 1
                
                x_values = scan.roiWidthX.flatten()
                z_values = scan.roiHeightZ.flatten()
                intensity_values = scan.intensity.flatten()
                
                encoder_unsigned = scan.encoderValue
                encoder = encoder_unsigned if encoder_unsigned < 2**31 else encoder_unsigned - 2**32
                
                if last_encoder is not None:
                    delta_encoder = encoder - last_encoder
                    delta_microns = delta_encoder * (ENC_SCALE_MM * 1000)
                    print(f"[{scanNo}] Encoder: {encoder}, ΔEncoder: {delta_encoder}, ΔDistance: {delta_microns} µm")
                else:
                    print(f"[{scanNo}] Encoder: {encoder}, ΔDistance: N/A")
                
                if last_encoder is None or encoder != last_encoder:
                    y_value = encoder * ENC_SCALE_MM
                    
                    # Create current scan points for ROS2 publishing
                    current_scan_points = []
                    
                    # Filter points by intensity and convert to meters (similar to C++ code)
                    for i in range(len(x_values)):
                        if intensity_values[i] >= INTENSITY_THRESHOLD:  # Skip low-intensity points
                            x = float(x_values[i] / 1000.0)  # Convert to meters
                            y = float(y_value / 1000.0)      # Convert to meters
                            z = float(z_values[i] / 1000.0)  # Convert to meters
                            
                            current_scan_points.append([x, y, z])
                    
                    # Add to all_points for PCD file
                    new_points = np.column_stack((x_values, np.full_like(x_values, y_value), z_values))
                    all_points.extend(new_points.tolist())
                    
                    # Publish point cloud every PUBLISH_INTERVAL scans or if we have enough points
                    if scanNo % PUBLISH_INTERVAL == 0 or len(current_scan_points) > 1000:
                        if len(current_scan_points) > 0:
                            pointcloud_publisher.publish_pointcloud(current_scan_points)
                
                last_encoder = encoder
                
            except SensorException as e:
                if "-1" in str(e):
                    logging.debug("No new profile available (likely waiting for encoder trigger).")
                    continue
                else:
                    raise
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping sensor...")
    
    # Publish final accumulated point cloud
    if len(all_points) > 0:
        # Convert all points to meters and publish
        final_points = []
        for pt in all_points:
            final_points.append([pt[0]/1000.0, pt[1]/1000.0, pt[2]/1000.0])
        pointcloud_publisher.publish_pointcloud(final_points)
    
    print(f"Saving {len(all_points)} points to PCD file...")
    with open(PCD_FILE, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(all_points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(all_points)}\n")
        f.write("DATA ascii\n")
        for pt in all_points:
            f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")
    
    sensor.write_data("SetAcquisitionStop")
    sensor.write_data("SetResetSettings")
    time.sleep(1)
    sensor.disconnect()
    
    # Cleanup ROS2
    pointcloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()