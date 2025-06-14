# -*- coding: utf-8 -*-

import os
import platform
import time
import logging
from dataclasses import dataclass

from ctypes import *

import numpy as np

logging.basicConfig(level=logging.DEBUG)

SENSOR_DISCONNECTED = 0
SENSOR_CONNECTED = 3

ENC_SCALE_MM = 0.02
PLY_FILE = "merged.ply"  # Changed from PCD to PLY


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


lib = cdll.LoadLibrary(os.path.join(os.getcwd(), 'EthernetScanner.dll')) if platform.system() == "Windows" else cdll.LoadLibrary(os.path.join(os.getcwd(), 'libEthernetScanner.so'))

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
    

def main():
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
    all_points = []  # Modified to store [x, y, z, intensity] tuples

    print("Sensor is running with encoder triggering. Press Ctrl+C to stop.")

    try:
        while True:
            try:
                scan = sensor.get_scanned_profile(timeout=1000)
                scanNo += 1
                x_values = scan.roiWidthX.flatten()
                z_values = scan.roiHeightZ.flatten()
                intensity_values = scan.intensity.flatten()  # Extract intensity values
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
                    # Create points with x, y, z, intensity
                    new_points = np.column_stack((x_values, np.full_like(x_values, y_value), z_values, intensity_values))
                    all_points.extend(new_points.tolist())

                last_encoder = encoder

            except SensorException as e:
                if "-1" in str(e):
                    logging.debug("No new profile available (likely waiting for encoder trigger).")
                    continue
                else:
                    raise

    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping sensor...")

    print(f"Saving {len(all_points)} points to PLY file...")
    with open(PLY_FILE, 'w') as f:
        # PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(all_points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property int intensity\n")
        f.write("end_header\n")
        
        # PLY data
        for pt in all_points:
            f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f} {int(pt[3])}\n")

    print(f"PLY file saved successfully: {PLY_FILE}")

    sensor.write_data("SetAcquisitionStop")
    sensor.write_data("SetResetSettings")
    time.sleep(1)
    sensor.disconnect()

if __name__ == "__main__":
    main()