#include <iostream>
#include <csignal>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <thread>
#include <exception>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <dlfcn.h>
#include "EthernetScannerSDK.h"
#include <csignal>
#include <iostream>



///-----DEFINING PARAMETERS-----///
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
volatile bool keep_running = true;

volatile sig_atomic_t stop = false;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nInterrupted by user. Stopping sensor..." << std::endl;
        stop = true;
    }
}
///-----STRUCT CLASSES-----///
struct UserIOState {
    int EA1;
    int EA2;
    int EA3;
    int EA4;
    int TTLEncA;
    int TTLEncB;
    int TTLEncC;
    UserIOState(int ea1, int ea2, int ea3, int ea4, int ttlA, int ttlB, int ttlC)
        : EA1(ea1), EA2(ea2), EA3(ea3), EA4(ea4), TTLEncA(ttlA), TTLEncB(ttlB), TTLEncC(ttlC) {}
    
    UserIOState() = default;
};
struct ScannedProfile {
    std::vector<float> roiWidthX;
    std::vector<float> roiHeightZ;
    std::vector<uint8_t> intensity;
    std::vector<float> signalWidth;
    unsigned int encoderValue;
    UserIOState userIOState;
    int pictureCounter;
    int scannedPoints;
};
struct CameraImage {
    std::vector<uint8_t> rawImageData;
    int imgWidth;
    int imgHeight;
    int imgOffsetX;
    int imgOffsetY;
    int imgStepX;
    int imgStepY;
};
//------CONVERTING INTO CTYPE VARIABLES------//
// Fixed function pointer declarations to match actual signatures
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

// Declare function pointer variables with different names to avoid conflicts
EthernetScanner_GetConnectStatus_ptr p_EthernetScanner_GetConnectStatus = nullptr;
EthernetScanner_GetXZIExtended_ptr p_EthernetScanner_GetXZIExtended = nullptr;
EthernetScanner_GetImage_ptr p_EthernetScanner_GetImage = nullptr;
EthernetScanner_Connect_ptr p_EthernetScanner_Connect = nullptr;
EthernetScanner_Disconnect_ptr p_EthernetScanner_Disconnect = nullptr;
EthernetScanner_GetDllFiFoState_ptr p_EthernetScanner_GetDllFiFoState = nullptr;
EthernetScanner_ResetDllFiFo_ptr p_EthernetScanner_ResetDllFiFo = nullptr;
EthernetScanner_GetVersion_ptr p_EthernetScanner_GetVersion = nullptr;
EthernetScanner_WriteData_ptr p_EthernetScanner_WriteData = nullptr;
EthernetScanner_ReadData_ptr p_EthernetScanner_ReadData = nullptr;

// Library handle
#ifdef _WIN32
HMODULE libHandle = nullptr;
#else
void* libHandle = nullptr;
#endif

// Function to load the library and assign function pointers
bool loadEthernetScannerLibrary() {
#ifdef _WIN32
    libHandle = LoadLibrary("EthernetScanner.dll");
#else
    // Try multiple possible library locations
    const char* library_paths[] = {
        "./libEthernetScanner.so",
        "/usr/local/lib/libEthernetScanner.so",
        "/usr/lib/libEthernetScanner.so",
        "libEthernetScanner.so"
    };
    
    for (const char* path : library_paths) {
        libHandle = dlopen(path, RTLD_LAZY);
        if (libHandle) {
            std::cout << "Successfully loaded library from: " << path << std::endl;
            break;
        }
    }
#endif
    
    if (!libHandle) {
        std::cerr << "Failed to load EthernetScanner library!" << std::endl;
        std::cerr << "Error: " << dlerror() << std::endl;
        return false;
    }

#ifdef _WIN32
    #define LOAD_SYMBOL(name) (name = (name##_ptr)GetProcAddress(libHandle, #name))
#else
    #define LOAD_SYMBOL(name) (p_##name = (name##_ptr)dlsym(libHandle, #name))
#endif

    // Load each function with error checking
    if (!LOAD_SYMBOL(EthernetScanner_Connect)) {
        std::cerr << "Failed to load EthernetScanner_Connect: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_Disconnect)) {
        std::cerr << "Failed to load EthernetScanner_Disconnect: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetConnectStatus)) {
        std::cerr << "Failed to load EthernetScanner_GetConnectStatus: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetXZIExtended)) {
        std::cerr << "Failed to load EthernetScanner_GetXZIExtended: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetImage)) {
        std::cerr << "Failed to load EthernetScanner_GetImage: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetDllFiFoState)) {
        std::cerr << "Failed to load EthernetScanner_GetDllFiFoState: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_ResetDllFiFo)) {
        std::cerr << "Failed to load EthernetScanner_ResetDllFiFo: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetVersion)) {
        std::cerr << "Failed to load EthernetScanner_GetVersion: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_WriteData)) {
        std::cerr << "Failed to load EthernetScanner_WriteData: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_ReadData)) {
        std::cerr << "Failed to load EthernetScanner_ReadData: " << dlerror() << std::endl;
        return false;
    }
    
    std::cout << "All functions loaded successfully!" << std::endl;
    return true;
}

// Function to unload the library
void unloadEthernetScannerLibrary() {
    if (libHandle) {
#ifdef _WIN32
        FreeLibrary(libHandle);
#else
        dlclose(libHandle);
#endif
        libHandle = nullptr;
        std::cout << "Library unloaded successfully" << std::endl;
    }
}
#define SENSOR_CONNECTED 3
#define SENSOR_DISCONNECTED 0

// Forward declaration of SensorException
class SensorException : public std::runtime_error {
public:
    explicit SensorException(const std::string& message)
        : std::runtime_error(message) {}
};

///----SENSOR CLASS----///
class Sensor {
public:
    std::string ip;
    std::string port;
    void* handle;
    static constexpr size_t read_buf_size = 128 * 1024;
    char read_buf[read_buf_size];
    int status;
    // Profile buffers
    std::unique_ptr<double[]> roiWidthX;
    std::unique_ptr<double[]> roiHeightZ;
    std::unique_ptr<int[]> intensity;
    std::unique_ptr<int[]> signalWidth;
    uint32_t encoderValue;
    uint32_t userIOState;
    int pictureCounter;
    // Camera image buffers
    std::unique_ptr<char[]> rawBuffer;
    int rawBufferSize;
    std::unique_ptr<uint32_t[]> width;
    std::unique_ptr<uint32_t[]> height;
    std::unique_ptr<uint32_t[]> offsetX;
    std::unique_ptr<uint32_t[]> offsetY;
    std::unique_ptr<uint32_t[]> stepX;
    std::unique_ptr<uint32_t[]> stepY;

    Sensor(const std::string& ipAddress, int portNumber)
        : ip(ipAddress), 
          port(std::to_string(portNumber)),
          handle(nullptr), 
          status(0),
          encoderValue(0),          // Initialize directly
          userIOState(0),           // Initialize directly  
          pictureCounter(0),        // Initialize directly
          rawBufferSize(0),
          roiWidthX(nullptr), 
          roiHeightZ(nullptr), 
          intensity(nullptr), 
          signalWidth(nullptr),
          rawBuffer(nullptr),
          width(nullptr), 
          height(nullptr), 
          offsetX(nullptr),
          offsetY(nullptr), 
          stepX(nullptr), 
          stepY(nullptr) {
        std::memset(read_buf, 0, read_buf_size);
    }
    void connect(int timeout = 0) {
        std::cout << "Attempting to connect with the sensor at " << ip << "...\n";
        
        // Check if function pointers are loaded
        if (!p_EthernetScanner_Connect) {
            throw std::runtime_error("EthernetScanner_Connect function not loaded!");
        }
        
        // Create mutable copies of the strings for the C API
        std::vector<char> ip_copy(ip.begin(), ip.end());
        ip_copy.push_back('\0');
        std::vector<char> port_copy(port.begin(), port.end());
        port_copy.push_back('\0');
        
        std::cout << "Calling EthernetScanner_Connect with IP: " << ip << ", Port: " << port << ", Timeout: " << timeout << std::endl;
        
        handle = p_EthernetScanner_Connect(ip_copy.data(), port_copy.data(), timeout);
        
        if (!handle) {
            std::cout << "Initial connection attempt returned null handle" << std::endl;
        } else {
            std::cout << "Got handle from EthernetScanner_Connect: " << handle << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto connectionPollingStart = std::chrono::steady_clock::now();
        while (!handle && std::chrono::steady_clock::now() - connectionPollingStart < std::chrono::milliseconds(timeout)) {
            // Busy wait (could also sleep a bit to avoid CPU burn)
        }
        
        std::cout << "Checking connection status..." << std::endl;
        int connectionStatus = get_connect_status();
        std::cout << "Connection status: " << connectionStatus << std::endl;
        
        if (connectionStatus != SENSOR_CONNECTED) {
            throw std::runtime_error("Failed to connect with the sensor; Connection status: " + std::to_string(connectionStatus));
        }
        std::cout << "Sensor has been connected successfully. Status: " << connectionStatus << "\n";
        allocate_memory();
    }
    void disconnect() {
        handle = p_EthernetScanner_Disconnect(handle);
        if (handle) {
            throw std::runtime_error("Failed to disconnect the sensor.");
        }
        std::cout << "Sensor has been disconnected successfully. Status: " << SENSOR_DISCONNECTED << "\n";
        deallocate_memory();
    }
    int get_connect_status() {
        if (!p_EthernetScanner_GetConnectStatus) {
            std::cerr << "EthernetScanner_GetConnectStatus function not loaded!" << std::endl;
            return SENSOR_DISCONNECTED;
        }
        
        if (!handle) {
            std::cout << "Handle is null, returning SENSOR_DISCONNECTED" << std::endl;
            return SENSOR_DISCONNECTED;
        }
        
        std::cout << "Calling EthernetScanner_GetConnectStatus with handle: " << handle << std::endl;
        p_EthernetScanner_GetConnectStatus(handle, &status);
        std::cout << "Raw status from GetConnectStatus: " << status << std::endl;
        
        int result = (status & SENSOR_CONNECTED) ? SENSOR_CONNECTED : SENSOR_DISCONNECTED;
        std::cout << "Processed connection status: " << result << std::endl;
        return result;
    }
    void allocate_memory() {
         std::cout << "Allocating memory for profile and camera image ...\n";
        // Profile buffers - using make_unique
        roiWidthX = std::make_unique<double[]>(SENSOR_BUFFERSIZEMAX);
    roiHeightZ = std::make_unique<double[]>(SENSOR_BUFFERSIZEMAX);
    intensity = std::make_unique<int[]>(SENSOR_BUFFERSIZEMAX);
    signalWidth = std::make_unique<int[]>(SENSOR_BUFFERSIZEMAX);
    encoderValue = 0;
    userIOState = 0;
    pictureCounter = 0;
    rawBuffer = std::make_unique<char[]>(SENSOR_BUFFERSIZEMAX);
    rawBufferSize = SENSOR_BUFFERSIZEMAX;
    width = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    height = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    offsetX = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    offsetY = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    stepX = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    stepY = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
        std::cout << "Memory has been allocated successfully\n";
    }
    void deallocate_memory() {
        // Profile buffers
        roiWidthX.reset();
        roiHeightZ.reset();
        intensity.reset();
        signalWidth.reset();
        
        // Primitive members don't need explicit deallocation
        encoderValue = 0;
        userIOState = 0;
        pictureCounter = 0;
        
        // Camera buffers
        rawBuffer.reset();
        width.reset();
        height.reset();
        offsetX.reset();
        offsetY.reset();
        stepX.reset();
        stepY.reset();
        rawBufferSize = 0;
        
        std::cout << "Memory deallocated successfully\n";
    }
    CameraImage get_camera_image(int timeout = 3000) {
        std::cout << "Reading camera image ...\n";

        int response = p_EthernetScanner_GetImage(
            handle,
            rawBuffer.get(),
            rawBufferSize,
            width.get(),
            height.get(),
            offsetX.get(),
            offsetY.get(),
            stepX.get(),
            stepY.get(),
            static_cast<uint32_t>(timeout)
        );

        if (response < 0) {
            throw SensorException("Failed to get camera image. Error code: " + std::to_string(response));
        }

        std::cout << "Camera image received successfully\n";

        uint32_t imgResolution = width[0] * height[0];

        // Copy rawBuffer to a vector or custom structure
        std::vector<uint8_t> imgData(rawBuffer.get(), rawBuffer.get() + imgResolution);

        CameraImage img;
        img.rawImageData = imgData;
        img.imgWidth = width[0];
        img.imgHeight = height[0];
        img.imgOffsetX = offsetX[0];
        img.imgOffsetY = offsetY[0];
        img.imgStepX = stepX[0];
        img.imgStepY = stepY[0];

        return img;
    }
    ScannedProfile get_scanned_profile(int timeout = 1000) {
        std::cout << "Reading scanned profile ...\n";

        // Deprecated parameters passed as NULL
        unsigned char* ucBufferRaw = nullptr;
        int iBufferRaw = 0;

        int response = p_EthernetScanner_GetXZIExtended(
            handle,
            roiWidthX.get(),
            roiHeightZ.get(),
            intensity.get(),
            signalWidth.get(),
            rawBufferSize,
            &encoderValue,              // Pass address of member variable
            (unsigned char*)&userIOState,  // Pass address of member variable
            timeout,
            ucBufferRaw,
            iBufferRaw,
            &pictureCounter 
        );

        if (response < 0) {
            throw SensorException("Failed to get scanned profile. Error code: " + std::to_string(response));
        }

        std::cout << "Profile received successfully for " << response << " points\n";

        UserIOState userIOStateStruct(
            (userIOState & 0b1),
            (userIOState & 0b10) >> 1,
            (userIOState & 0b100) >> 2,
            (userIOState & 0b1000) >> 3,
            (userIOState & 0b10000) >> 4,
            (userIOState & 0b100000) >> 5,
            (userIOState & 0b1000000) >> 6
        );

        // Convert double vectors to float vectors for ScannedProfile struct
        std::vector<float> roiWidthXVec(response);
        std::vector<float> roiHeightZVec(response);
        std::vector<uint8_t> intensityVec(response);
        std::vector<float> signalWidthVec(response);

        for (int i = 0; i < response; ++i) {
            roiWidthXVec[i] = static_cast<float>(roiWidthX[i]);
            roiHeightZVec[i] = static_cast<float>(roiHeightZ[i]);
            intensityVec[i] = static_cast<uint8_t>(intensity[i]);
            signalWidthVec[i] = static_cast<float>(signalWidth[i]);
        }

        return ScannedProfile{
            roiWidthXVec,
            roiHeightZVec,
            intensityVec,
            signalWidthVec,
            encoderValue,            
            userIOStateStruct,
            pictureCounter,           
            response
        };
    }

    std::string get_dll_version() {
        int response = p_EthernetScanner_GetVersion(reinterpret_cast<unsigned char*>(read_buf), read_buf_size);

        if (response < 0) {
            throw SensorException("Failed to get DLL version. Error code: " + std::to_string(response));
        }

        return std::string(read_buf);
    }
    std::string read_data(const std::string& command, int cacheTime = 0) {
        std::vector<char> cmdBuffer(command.begin(), command.end());
        cmdBuffer.push_back('\0');  

        int response = p_EthernetScanner_ReadData(
            handle,
            cmdBuffer.data(),
            read_buf,
            read_buf_size,
            cacheTime
        );

        if (response == SENSOR_READDATAOK) {
            std::cout << "Sent Read Data command " << command << " -> Response: " << read_buf << " (Sensor_READDATAOK)\n";
        } else {
            throw SensorException("Failed to read data. Error code: " + std::to_string(response));
        }

        return std::string(read_buf);
    }

    int write_data(const std::string& command) {
        std::vector<char> cmdBuffer(command.begin(), command.end());
        cmdBuffer.push_back('\0'); 

        int response = p_EthernetScanner_WriteData(
            handle,
            cmdBuffer.data(),
            static_cast<int>(command.size())
        );

        int responseExpected = static_cast<int>(command.size());
        if (response == responseExpected) {
            std::cout << "Sent Write Data command " << command << " -> Response: " << response << " (len(command))\n";
        } else if (response > 0 && response < responseExpected) {
            std::cerr << "Warning: Sent Write Data command " << command << " -> Response: " << response << "; Expected: " << responseExpected << " (len(command))\n";
        } else {
            throw SensorException("Failed to write data. Error code: " + std::to_string(response));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return response;
    }
    void publish_scan_data(
        rclcpp::Node::SharedPtr node, 
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
        const std::vector<float>& x_vals, 
        float y_val, 
        const std::vector<float>& z_vals,
        int64_t encoder_val) {

        static sensor_msgs::msg::PointCloud2 msg;  // Static to avoid reallocation
        static bool first_time = true;

        if (first_time) {
            // Set up fields: x, y, z as FLOAT32
            sensor_msgs::PointCloud2Modifier mod(msg);
            mod.setPointCloud2FieldsByString(1, "xyz");
            msg.is_bigendian = false;
            msg.is_dense = false;  // Set to true if no invalid points
            first_time = false;
        }

        // Update header
        msg.header.stamp = node->get_clock()->now();
        msg.header.frame_id = "map";

        // Set dimensions
        msg.height = 1;
        msg.width = x_vals.size();

        // Set point step (size of one point in bytes: 3 floats = 12 bytes)
        msg.point_step = 3 * sizeof(float);

        // Set row step (size of one row in bytes)
        msg.row_step = msg.point_step * msg.width;

        // Resize data array
        msg.data.resize(msg.row_step * msg.height);

        // Fill in the data (x, y, z as floats)
        float* data_ptr = reinterpret_cast<float*>(msg.data.data());
        for (size_t i = 0; i < x_vals.size(); ++i) {
            data_ptr[i * 3]     = x_vals[i] / 1000.0f;     // Convert mm to meters
            data_ptr[i * 3 + 1] = y_val / 1000.0f;         // Same y for all points
            data_ptr[i * 3 + 2] = z_vals[i] / 1000.0f;     // Convert mm to meters
        }

        // Publish the message
        pub->publish(msg);
    }
};

int main(int argc, char* argv[]) {
    std::cout << "Starting encoder test program..." << std::endl;
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    auto node = rclcpp::Node::make_shared("wecat3d_runtime_node");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  // RViz expects Reliable
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.history(rclcpp::HistoryPolicy::KeepLast);
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wecat3d/pointcloud", qos);
    auto merged_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!loadEthernetScannerLibrary()) {
        std::cerr << "Failed to load EthernetScanner library!" << std::endl;
        return 1;
    }

    try {
        std::cout << "Creating sensor object..." << std::endl;
        Sensor sensor("192.168.100.1", 32001);  
        
        std::cout << "Attempting to connect to sensor..." << std::endl;
        sensor.connect(0); 
        sensor.write_data("SetHeartbeat=1000");

        // sensor.write_data("SetAcquisitionStop");
        // std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
    
        sensor.write_data("SetExposureTime=750");

        sensor.write_data("SetTriggerSource=2");
        sensor.write_data("SetTriggerEncoderStep=20");
        sensor.write_data("SetEncoderTriggerFunction=2");
        sensor.write_data("ResetEncoder");
        sensor.write_data("SetRangeImageNrProfiles=1");
        sensor.write_data("SetAcquisitionStart");

        int scanNo = 0;
        int64_t last_encoder = std::numeric_limits<int64_t>::min();

        std::vector<std::vector<float>> all_points;

        std::cout << "Sensor is running with encoder triggering. Press Ctrl+C to stop." << std::endl;

        std::signal(SIGINT, signal_handler);

    while (!stop) {
        rclcpp::spin_some(node);
        try {
            ScannedProfile scan = sensor.get_scanned_profile(1000);
            scanNo++;
            
            const std::vector<float>& x_values = scan.roiWidthX;
            const std::vector<float>& z_values = scan.roiHeightZ;
            uint32_t encoder_unsigned = scan.encoderValue;

            int64_t encoder = static_cast<int64_t>(encoder_unsigned);
            if (encoder_unsigned >= static_cast<uint32_t>(1ULL << 31)) {
                encoder -= static_cast<int64_t>(1ULL << 32);
            }

            if (last_encoder != std::numeric_limits<int64_t>::min()) {
                int64_t delta_encoder = encoder - last_encoder;
                double delta_microns = delta_encoder * (ENC_SCALE_MM * 1000.0);
                std::cout << "[" << scanNo << "] Encoder: " << encoder
                        << ", ΔEncoder: " << delta_encoder
                        << ", ΔDistance: " << delta_microns << " µm" << std::endl;
            }
            
            if (last_encoder == std::numeric_limits<int64_t>::min() || encoder != last_encoder) {
                
                double y_value = encoder * ENC_SCALE_MM;  
                
                // Create PCL point cloud for this scan
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                    cur_cloud->width = x_values.size();
                    cur_cloud->height = 1;
                    cur_cloud->is_dense = false;
                    cur_cloud->points.resize(x_values.size());
                    
                    for (size_t i = 0; i < x_values.size(); ++i) {
                        cur_cloud->points[i].x = x_values[i];     
                        cur_cloud->points[i].y = y_value;         
                        cur_cloud->points[i].z = z_values[i];     
                    }
                    
                    *merged_cloud += *cur_cloud;
                    
                    // Save PCD file periodically
                    if (scanNo % 500 == 0) { 
                        pcl::io::savePCDFileBinaryCompressed("merged_encoder_output.pcd", *merged_cloud);
                        std::cout << "Saved PCD with " << merged_cloud->points.size() << " points" << std::endl;
                    }
                    
                    // ACTUALLY CALL the publish function here
                    sensor.publish_scan_data(node, pub, x_values, y_value, z_values, encoder);
                    
                    RCLCPP_INFO(node->get_logger(), 
                            "Published %zu points | Encoder: %ld | Y=%.3f mm", 
                            x_values.size(), encoder, y_value);
                }
            
            last_encoder = encoder;
            
        } catch (const SensorException& e) {
            if (std::string(e.what()).find("-1") != std::string::npos) {
                continue;
            } else {
                throw;
            }
        }
    }
    pcl::io::savePCDFileBinaryCompressed("final_merged_encoder_output.pcd", *merged_cloud);
    std::cout << "Final PCD saved with " << merged_cloud->points.size() << " total points" << std::endl;
            std::cout << "Getting connection status..." << std::endl;
            int status = sensor.get_connect_status();
            if (status == SENSOR_CONNECTED) {
                std::cout << "Sensor is confirmed to be connected." << std::endl;
            } else {
                std::cout << "Sensor is NOT connected after attempt." << std::endl;
            }
            
            std::cout << "Disconnecting sensor..." << std::endl;
            sensor.write_data("SetAcquisitionStop");
            sensor.write_data("SetResetSettings");
            sensor.disconnect();
            rclcpp::shutdown();
            std::cout << "Sensor disconnected. Shutdown complete.\n";
        }
        catch (const std::exception& e) {
            std::cerr << "Exception occurred: " << e.what() << std::endl;
            unloadEthernetScannerLibrary();
            return 1;
        }
        unloadEthernetScannerLibrary();
        return 0;
    

}