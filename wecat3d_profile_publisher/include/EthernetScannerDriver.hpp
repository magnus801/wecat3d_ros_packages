#ifndef ETHERNETSCANNERDRIVER_HPP
#define ETHERNETSCANNERDRIVER_HPP

#include <iostream>
#include <csignal>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <thread>
#include <exception>
#include <string>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <limits>

// Constants and Macros
#define WIDTH_X 1280
#define HEIGHT_Z 1024
#define SENSOR_BUFFERSIZEMAX 4202500
#define ENC_SCALE_MM 0.02
#define PCD_FILE "merged.pcd"
#define SENSOR_OK 0
#define SENSOR_ERROR -1
#define SENSOR_GETINFOSMALLBUFFER -2
#define SENSOR_GETINFONOVALIDINFO -3
#define SENSOR_GETINFOINVALIDXML -4
#define SENSOR_GETXZINONEWSCAN -1
#define SENSOR_GETXZIINVALIDLINDATA -2
#define SENSOR_GETXZIINVALIDBUFFER -3
#define SENSOR_READDATAOK 0
#define SENSOR_READDATASMALLBUFFER -1
#define SENSOR_READDATANOTSUPPORTEDMODE -2
#define SENSOR_READDATAFEATURENOTDEFINED -3
#define SENSOR_READDATANOSCAN -4
#define SENSOR_READDATAFAILED -5
#define SENSOR_WRITEDATAINVALIDSOCKET -1
#define SENSOR_INVALIDHANDLE -1000
#define SENSOR_GETINFOSMALLERBUFFER -2
#define SENSOR_CONNECTED 3
#define SENSOR_DISCONNECTED 0

extern volatile bool keep_running;
extern volatile sig_atomic_t stop;

// Signal handler
void signal_handler(int signal);

// UserIOState struct
struct UserIOState {
    int EA1;
    int EA2;
    int EA3;
    int EA4;
    int TTLEncA;
    int TTLEncB;
    int TTLEncC;
    UserIOState(int ea1, int ea2, int ea3, int ea4, int ttlA, int ttlB, int ttlC);
    UserIOState() = default;
};

// ScannedProfile struct
struct ScannedProfile {
    std::vector<float> roiWidthX;
    std::vector<float> roiHeightZ;
    std::vector<uint8_t> intensity;
    std::vector<float> signalWidth;
    unsigned int encoderValue;
    UserIOState userIOState;
    int pictureCounter;
    int scannedPoints;
    ScannedProfile();
    ScannedProfile(int x, int z);
    ScannedProfile(const ScannedProfile& other) = default;
    ScannedProfile(ScannedProfile&& other) noexcept;
    ScannedProfile& operator=(const ScannedProfile& other) = default;
    ScannedProfile& operator=(ScannedProfile&& other) noexcept;
    ~ScannedProfile() = default;
};

// CameraImage struct
struct CameraImage {
    std::vector<uint8_t> rawImageData;
    int imgWidth;
    int imgHeight;
    int imgOffsetX;
    int imgOffsetY;
    int imgStepX;
    int imgStepY;
};

// EthernetScanner SDK function pointers
typedef void (*EthernetScanner_GetConnectStatus_ptr)(void*, int*);
typedef int  (*EthernetScanner_GetXZIExtended_ptr)(void*, double*, double*, int*, int*, int, unsigned int*, unsigned char*, int, unsigned char*, int, int*);
typedef int  (*EthernetScanner_GetImage_ptr)(void*, char*, int, unsigned int*, unsigned int*, unsigned int*, unsigned int*, unsigned int*, unsigned int*, unsigned int);
typedef void* (*EthernetScanner_Connect_ptr)(char*, char*, int);
typedef void* (*EthernetScanner_Disconnect_ptr)(void*);
typedef int   (*EthernetScanner_GetDllFiFoState_ptr)(void*);
typedef int   (*EthernetScanner_ResetDllFiFo_ptr)(void*);
typedef int   (*EthernetScanner_GetVersion_ptr)(unsigned char*, int);
typedef int   (*EthernetScanner_WriteData_ptr)(void*, char*, int);
typedef int   (*EthernetScanner_ReadData_ptr)(void*, char*, char*, int, int);

// Function pointer variables
extern EthernetScanner_GetConnectStatus_ptr p_EthernetScanner_GetConnectStatus;
extern EthernetScanner_GetXZIExtended_ptr p_EthernetScanner_GetXZIExtended;
extern EthernetScanner_GetImage_ptr p_EthernetScanner_GetImage;
extern EthernetScanner_Connect_ptr p_EthernetScanner_Connect;
extern EthernetScanner_Disconnect_ptr p_EthernetScanner_Disconnect;
extern EthernetScanner_GetDllFiFoState_ptr p_EthernetScanner_GetDllFiFoState;
extern EthernetScanner_ResetDllFiFo_ptr p_EthernetScanner_ResetDllFiFo;
extern EthernetScanner_GetVersion_ptr p_EthernetScanner_GetVersion;
extern EthernetScanner_WriteData_ptr p_EthernetScanner_WriteData;
extern EthernetScanner_ReadData_ptr p_EthernetScanner_ReadData;

// Library handle
#ifdef _WIN32
extern HMODULE libHandle;
#else
extern void* libHandle;
#endif

// Library loading/unloading
bool loadEthernetScannerLibrary();
void unloadEthernetScannerLibrary();

// Exception class
class SensorException : public std::runtime_error {
public:
    explicit SensorException(const std::string& message);
};

// Sensor class
class Sensor {
public:
    std::string ip;
    std::string port;
    void* handle;
    static constexpr size_t read_buf_size = 128 * 1024;
    std::unique_ptr<char[]> read_buf;
    int status;
    std::unique_ptr<double[]> roiWidthX;
    std::unique_ptr<double[]> roiHeightZ;
    std::unique_ptr<int[]> intensity;
    std::unique_ptr<int[]> signalWidth;
    std::unique_ptr<uint32_t> encoderValue;
    std::unique_ptr<uint32_t> userIOState;
    std::unique_ptr<int> pictureCounter;
    std::unique_ptr<char[]> rawBuffer;
    size_t rawBufferSize;
    std::unique_ptr<uint32_t[]> width;
    std::unique_ptr<uint32_t[]> height;
    std::unique_ptr<uint32_t[]> offsetX;
    std::unique_ptr<uint32_t[]> offsetY;
    std::unique_ptr<uint32_t[]> stepX;
    std::unique_ptr<uint32_t[]> stepY;

    Sensor(const std::string& ipAddress, int portNumber);
    ~Sensor() = default;
    void connect(int timeout = 0);
    void disconnect();
    int get_connect_status();
    void allocate_memory();
    void deallocate_memory();
    CameraImage get_camera_image(int timeout = 3000);
    void get_scanned_profile(ScannedProfile& profile, int timeout = 1000);
    std::string get_dll_version();
    std::string read_data(const std::string& command, int cacheTime = 0);
    int write_data(const std::string& command);
};

#endif // ETHERNETSCANNERDRIVER_HPP
