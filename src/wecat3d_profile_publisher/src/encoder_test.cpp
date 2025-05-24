#include <iostream>
#include <cstring>
#include <csignal>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>    // <-- include PLY writer

#include "EthernetScannerSDK.h"
#include "EthernetScannerSDKDefine.h"

#define WIDTH_X      1280
#define HEIGHT_Z     1024
#define BUFFER_SIZE  WIDTH_X

volatile bool keepRunning = true;
void handleSigint(int) {
    keepRunning = false;
    std::cout << "\nCtrl+C received. Stopping...\n";
}

void sendCommand(void* handle, const char* cmd) {
    // send trailing \r, but log without it
    std::string out = std::string(cmd) + "\r";
    int ret = EthernetScanner_WriteData(handle, (char*)out.c_str(), out.size());
    std::string clean(cmd);
    if (!clean.empty() && clean.back() == '\r') clean.pop_back();
    std::cout << "Sent: " << clean << "  (code " << ret << ")\n";
}

void readProperty(void* handle, const char* name) {
    char buffer[ETHERNETSCANNER_BUFFERSIZEMAX] = {0};
    int ret = EthernetScanner_ReadData(handle, (char*)name, buffer,
                                       ETHERNETSCANNER_BUFFERSIZEMAX, 0);
    if (ret == ETHERNETSCANNER_READDATAOK)
        std::cout << "  " << name << " = " << buffer << "\n";
    else
        std::cout << "  Failed to read " << name << " (code " << ret << ")\n";
}

int main(int argc, char* argv[]) {
    // --- ROS2 init & SIGINT handler ---
    rclcpp::init(argc, argv);
    signal(SIGINT, handleSigint);

    auto node = rclcpp::Node::make_shared("wecat3d_profile_publisher");
    auto pub  = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "wecat3d/pointcloud", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    // --- Connect to the sensor ---
    char ip[]   = "192.168.100.1";
    char port[] = "32001";
    std::cout << "Connecting to sensor at " << ip << ":" << port << "...\n";
    void* handle = EthernetScanner_Connect(ip, port, 1000);
    if (!handle) {
        std::cerr << "Connection failed.\n";
        return -1;
    }
    int status = 0, tries = 0;
    while (status != 3 && tries++ < 10) {
        usleep(200000);
        EthernetScanner_GetConnectStatus(handle, &status);
    }
    if (status != 3) {
        std::cerr << "Sensor not ready (status=" << status << ")\n";
        EthernetScanner_Disconnect(handle);
        return -1;
    }

    // --- Configure sensor for one-profile-per-tick ---
    sendCommand(handle, "SetTriggerSource=2");
    sendCommand(handle, "SetTriggerEncoderStep=1");
    sendCommand(handle, "SetEncoderTriggerFunction=1");
    sendCommand(handle, "ResetEncoder");

    sendCommand(handle, "SetRangeImageNrProfiles=1");
    sendCommand(handle, "SetSignalSelection=1");
    sendCommand(handle, "SetROI1WidthX=1280");
    sendCommand(handle, "SetROI1HeightZ=1024");

    sendCommand(handle, "SetAutoExposureMode=1");
    sendCommand(handle, "SetAutoExposureTimeMin=100");
    sendCommand(handle, "SetAutoExposureTimeMax=1000");
    sendCommand(handle, "SetAutoExposureIntensityRangeMin=0");
    sendCommand(handle, "SetAutoExposureIntensityRangeMax=800");
    sendCommand(handle, "SetAutoExposureRangeXMin=100");
    sendCommand(handle, "SetAutoExposureRangeXMax=1280");

    // --- Start acquisition (doc §11.2) ---
    sendCommand(handle, "SetAcquisitionStop");
    usleep(200000);
    sendCommand(handle, "SetInitializeAcquisition");
    sendCommand(handle, "SetLinearizationMode=1");
    EthernetScanner_ResetDllFiFo(handle);
    sendCommand(handle, "SetAcquisitionStart");

    std::cout << "\nSensor parameters:\n";
    readProperty(handle, "TriggerSource");
    readProperty(handle, "TriggerEncoderStep");
    readProperty(handle, "EncoderTriggerFunction");
    readProperty(handle, "RangeImageNrProfiles");
    readProperty(handle, "ROI1WidthX");
    readProperty(handle, "ROI1HeightZ");
    readProperty(handle, "AutoExposureIntensityRangeMin");
    readProperty(handle, "AutoExposureIntensityRangeMax");

    std::cout << "\nStreaming profiles... (Ctrl+C to stop)\n";

    int32_t baseEnc = 0, lastRaw = 0;
    bool initialized = false;

    // per-profile buffers
    double  xBuf[BUFFER_SIZE], zBuf[BUFFER_SIZE];
    int     iBuf[BUFFER_SIZE], wBuf[BUFFER_SIZE];
    unsigned int rawEnc;
    unsigned char usrIO;
    int picCount;

    // --- Main loop: collect into merged_cloud + publish ROS2 ---
    while (keepRunning && rclcpp::ok()) {
        int n = EthernetScanner_GetXZIExtended(
            handle,
            xBuf, zBuf,
            iBuf, wBuf,
            BUFFER_SIZE,
            &rawEnc,
            &usrIO,
            500, nullptr, 0, &picCount
        );
        if (n != WIDTH_X) {
            // no new profile within timeout
            continue;
        }

        int32_t raw = static_cast<int32_t>(rawEnc);
        if (!initialized) {
            baseEnc    = lastRaw = raw;
            initialized = true;
            std::cout << "Base encoder = " << baseEnc << "\n";
        }
        int32_t rel = raw - baseEnc;
        int32_t d   = raw - lastRaw;
        lastRaw = raw;
        float y_m  = rel / 1000.0f;

        std::cout << "Raw=" << raw
                  << " | Rel=" << rel
                  << " | Δ=" << d
                  << " | y(m)=" << y_m << "\n";

        // append to merged_cloud
        for (int i = 0; i < n; ++i) {
            pcl::PointXYZ pt;
            pt.x = static_cast<float>(xBuf[i]) / 1000.0f;
            pt.y = y_m;
            pt.z = static_cast<float>(zBuf[i]) / 1000.0f;
            merged_cloud->push_back(pt);
        }

        // publish as ROS2 PointCloud2
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp    = node->now();
        msg.header.frame_id = "map";
        msg.height          = 1;
        msg.width           = n;
        msg.is_dense        = false;
        msg.is_bigendian    = false;

        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(n);

        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

        for (int i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
            *it_x = merged_cloud->points[ merged_cloud->size() - n + i ].x;
            *it_y = merged_cloud->points[ merged_cloud->size() - n + i ].y;
            *it_z = merged_cloud->points[ merged_cloud->size() - n + i ].z;
        }

        pub->publish(msg);
        rclcpp::spin_some(node);
        usleep(100000);
    }

    // --- End acquisition + save PLY once ---
    sendCommand(handle, "SetAcquisitionStop");
    usleep(200000);

    if (merged_cloud->empty()) {
        std::cerr << "No points collected, skipping PLY save.\n";
    } else {
        const std::string ply_file = "merged.ply";
        if (pcl::io::savePLYFileASCII(ply_file, *merged_cloud) == 0) {
            std::cout << "Saved " << merged_cloud->size()
                      << " points to " << ply_file << " (ASCII PLY)\n";
        } else {
            std::cerr << "Error saving PLY file!\n";
        }
    }

    // --- Cleanup ---
    EthernetScanner_Disconnect(handle);
    std::cout << "\nDisconnected.\n";
    rclcpp::shutdown();
    return 0;
}
