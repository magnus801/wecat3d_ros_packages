#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "EthernetScannerSDK.h"
#include "EthernetScannerSDKDefine.h"

#define WIDTH_X    1280
#define HEIGHT_Z   100
#define BUFFER_SIZE (WIDTH_X * HEIGHT_Z)

volatile bool keepRunning = true;

void handleSigint(int /*sig*/) {
    std::cout << "\nCtrl+C received. Stopping...\n";
    keepRunning = false;
}

void sendCommand(void* handle, const char* cmd) {
    int result = EthernetScanner_WriteData(handle, (char*)cmd, strlen(cmd));
    std::cout << "Sent: " << cmd << " (code " << result << ")\n";
}

void readProperty(void* handle, const char* name) {
    char buffer[ETHERNETSCANNER_BUFFERSIZEMAX] = {0};
    int result = EthernetScanner_ReadData(handle, (char*)name, buffer, ETHERNETSCANNER_BUFFERSIZEMAX, 0);
    if (result == 0)
        std::cout << " " << name << " = " << buffer << "\n";
    else
        std::cout << " Failed to read " << name << " (code " << result << ")\n";
}

int main(int argc, char* argv[]) {
    // --- ROS2 init & signal handler ---
    rclcpp::init(argc, argv);
    signal(SIGINT, handleSigint);

    auto node = rclcpp::Node::make_shared("wecat3d_profile_publisher");
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("wecat3d/pointcloud", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // --- Connect to sensor ---
    char ip[]   = "192.168.100.1";
    char port[] = "32001";
    std::cout << "Connecting to weCat3D at " << ip << ":" << port << "...\n";
    void* handle = EthernetScanner_Connect(ip, port, 1000);
    if (!handle) {
        std::cerr << "Connection failed.\n";
        return -1;
    }

    int status = 0, attempts = 0;
    while (status != 3 && attempts++ < 10) {
        usleep(200000);
        EthernetScanner_GetConnectStatus(handle, &status);
    }
    if (status != 3) {
        std::cerr << "Sensor not ready. Status: " << status << "\n";
        EthernetScanner_Disconnect(handle);
        return -1;
    }

    // --- Configure sensor ---
    sendCommand(handle, "SetTriggerSource=2 \r");              // encoder trigger
    sendCommand(handle, "SetTriggerEncoderStep=1\r");         // every 5 ticks
    sendCommand(handle, "SetEncoderTriggerFunction=1\r");     // quadrature decoding
    sendCommand(handle, "ResetEncoder\r");

    sendCommand(handle, "SetRangeImageNrProfiles=1\r");
    sendCommand(handle, "SetSignalSelection=1\r");

    sendCommand(handle, "SetAutoExposureMode=0\r");
    sendCommand(handle, "SetExposureTime=150\r");  

    sendCommand(handle, "SetAcquisitionStop\r");
    usleep(500000);
    sendCommand(handle, "SetLinearizationMode=1\r");
    sendCommand(handle, "SetInitializeAcquisition\r");
    EthernetScanner_ResetDllFiFo(handle);
    sendCommand(handle, "SetAcquisitionStart\r");

    std::cout << "\nReading parameters from sensor:\n";
    readProperty(handle, "TriggerSource");
    readProperty(handle, "TriggerEncoderStep");
    readProperty(handle, "EncoderTriggerFunction");
    readProperty(handle, "RangeImageNrProfiles");
    readProperty(handle, "ROI1WidthX");
    readProperty(handle, "ROI1HeightZ");
    readProperty(handle, "AutoExposureIntensityRangeMin");
    readProperty(handle, "AutoExposureIntensityRangeMax");

    std::cout << "\nStreaming profiles... (Ctrl+C to stop)\n";

    // --- Encoder tracking variables ---
    int32_t baseEncoder = 0;
    int32_t lastRaw     = 0;
    bool    initialized = false;

    while (keepRunning && rclcpp::ok()) {
        // buffers for profile data
        double xBuffer[BUFFER_SIZE] = {0};
        double zBuffer[BUFFER_SIZE] = {0};
        int intensityBuffer[BUFFER_SIZE]     = {0};
        int signalWidthBuffer[BUFFER_SIZE]   = {0};
        unsigned int encoderRaw               = 0;
        unsigned char usrIO                   = 0;
        int pictureCount                      = 0;

        int numPoints = EthernetScanner_GetXZIExtended(
            handle,
            xBuffer, zBuffer,
            intensityBuffer, signalWidthBuffer,
            BUFFER_SIZE,
            &encoderRaw,
            &usrIO,
            1000, nullptr, 0, &pictureCount
        );

        if (numPoints > 0) {
            // cast raw, initialize baseline
            int32_t raw = static_cast<int32_t>(encoderRaw);
            if (!initialized) {
                baseEncoder = raw;
                lastRaw     = raw;
                initialized = true;
                std::cout << "Base encoder: " << baseEncoder << "\n\n";
            }

            // compute relative and delta
            int32_t relative = raw - baseEncoder;
            int32_t delta    = raw - lastRaw;
            lastRaw = raw;

            // y coordinate in meters (relative ticks / 1000)
            float y_val = static_cast<float>(relative) / 1000.0f;

            // print encoder diagnostics
            std::cout
                << "Raw: "      << raw
                << " | Rel: "   << relative
                << " | Δ: "     << delta
                << " | y(m): "  << y_val
                << std::endl;

            // --- build & publish PointCloud2 ---
            sensor_msgs::msg::PointCloud2 cloud_msg;
            cloud_msg.header.stamp    = node->now();
            cloud_msg.header.frame_id = "map";
            cloud_msg.height          = 1;
            cloud_msg.width           = numPoints;
            cloud_msg.is_dense        = false;
            cloud_msg.is_bigendian    = false;

            sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(numPoints);

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            current_cloud->width  = numPoints;
            current_cloud->height = 1;
            current_cloud->is_dense = false;
            current_cloud->points.resize(numPoints);

            for (int i = 0; i < numPoints; ++i, ++iter_x, ++iter_y, ++iter_z) {
                float x = static_cast<float>(xBuffer[i]) / 1000.0f;
                float z = static_cast<float>(zBuffer[i]) / 1000.0f;
                *iter_x = x;
                *iter_y = y_val;
                *iter_z = z;

                current_cloud->points[i].x = x;
                current_cloud->points[i].y = y_val;
                current_cloud->points[i].z = z;
            }

            *merged_cloud += *current_cloud;
            pcl::io::savePCDFileASCII("merged.pcd", *merged_cloud);
            publisher->publish(cloud_msg);

            RCLCPP_INFO(node->get_logger(),
                "Published %d pts | raw=%d rel=%d Δ=%d",
                numPoints, raw, relative, delta
            );
        }

        rclcpp::spin_some(node);
        usleep(100000);
    }
    sendCommand(handle, "SetAcquisitionStop\r");
    usleep(500000);

    // --- clean up ---
    EthernetScanner_Disconnect(handle);
    std::cout << "\nDisconnected.\n";
    rclcpp::shutdown();
    return 0;
}

