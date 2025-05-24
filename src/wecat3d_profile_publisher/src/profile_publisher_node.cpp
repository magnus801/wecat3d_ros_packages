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

#define WIDTH_X 1280
#define HEIGHT_Z 100
#define BUFFER_SIZE (WIDTH_X * HEIGHT_Z)

volatile bool keepRunning = true;

void handleSigint(int sig) {
    std::cout << "\nCtrl+C received. Stopping...\n";
    keepRunning = false;
}

void sendCommand(void* handle, const char* cmd) {
    int result = EthernetScanner_WriteData(handle, (char*)cmd, strlen(cmd));
    std::cout << " (code " << result << ") to send: " << cmd;
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
    rclcpp::init(argc, argv);
    signal(SIGINT, handleSigint);
    auto node = rclcpp::Node::make_shared("wecat3d_profile_publisher");
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("wecat3d/pointcloud", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    char ip[] = "192.168.100.1";
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

    sendCommand(handle, "SetTriggerSource=2\r");                 
    sendCommand(handle, "SetTriggerEncoderStep=5\r");           
    sendCommand(handle, "SetEncoderTriggerFunction=0\r");     
    sendCommand(handle, "ResetEncoder\r");
    sendCommand(handle, "SetRangeImageNrProfiles=5\r");       
    sendCommand(handle, "SetSignalSelection=3\r");               
    sendCommand(handle, "SetROI1WidthX=1280\r");
    sendCommand(handle, "SetROI1HeightZ=1024\r");
    sendCommand(handle, "SetAutoExposureMode=1\r");
    sendCommand(handle, "SetAutoExposureTimeMin=100\r");
    sendCommand(handle, "SetAutoExposureTimeMax=800\r");
    sendCommand(handle, "SetAutoExposureIntensityRangeMin=100\r");
    sendCommand(handle, "SetAutoExposureIntensityRangeMax=180\r");
    sendCommand(handle, "SetAutoExposureRangeXMin=0\r");
    sendCommand(handle, "SetAutoExposureRangeXMax=1279\r");
    sendCommand(handle, "SetAcquisitionStop\r");
    usleep(200000);
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

    int32_t lastEncoder = -1;

    while (keepRunning && rclcpp::ok()) {
        double xBuffer[BUFFER_SIZE] = {0};
        double zBuffer[BUFFER_SIZE] = {0};
        int intensityBuffer[BUFFER_SIZE] = {0};
        int signalWidthBuffer[BUFFER_SIZE] = {0};
        unsigned int encoder = 0;
        unsigned char usrIO = 0;
        int pictureCount = 0;

        int numPoints = EthernetScanner_GetXZIExtended(
            handle,
            xBuffer, zBuffer,
            intensityBuffer, signalWidthBuffer,
            BUFFER_SIZE,
            &encoder,
            &usrIO,
            1000, nullptr, 0, &pictureCount
        );

        if (numPoints > 0 && encoder != lastEncoder) {
            lastEncoder = encoder;
            float y_val = static_cast<float>(encoder) / 1000.0f;

            // Create PointCloud2 message
            sensor_msgs::msg::PointCloud2 cloud_msg;
            cloud_msg.header.stamp = node->now();
            cloud_msg.header.frame_id = "map";
            cloud_msg.height = 1;
            cloud_msg.width = numPoints;
            cloud_msg.is_dense = false;
            cloud_msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(numPoints);

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            current_cloud->width = numPoints;
            current_cloud->height = 1;
            current_cloud->is_dense = false;
            current_cloud->points.resize(numPoints);

            for (int i = 0; i < numPoints; ++i, ++iter_x, ++iter_y, ++iter_z) {
                float x = xBuffer[i] / 1000.0f;
                float y = y_val;
                float z = zBuffer[i] / 1000.0f;

                *iter_x = x;
                *iter_y = y;
                *iter_z = z;

                current_cloud->points[i].x = x;
                current_cloud->points[i].y = y;
                current_cloud->points[i].z = z;
            }

            // Append to merged cloud and save
            *merged_cloud += *current_cloud;
            pcl::io::savePCDFileASCII("merged.pcd", *merged_cloud);

            publisher->publish(cloud_msg);
            RCLCPP_INFO(node->get_logger(), "Published %d points and saved to merged.pcd | Encoder: %d", numPoints, encoder);
        } else if (encoder == lastEncoder) {
            std::cout << " No encoder motion detected.\n";
        } else {
            std::cout << " No valid profile received (code: " << numPoints << ")\n";
        }

        rclcpp::spin_some(node);
        usleep(100000);
    }

    handle = EthernetScanner_Disconnect(handle);
    if (handle == NULL) {
        std::cout << " Disconnected successfully.\n";
    } else {
        std::cout << " Failed to disconnect.\n";
    }

    rclcpp::shutdown();
    return 0;
}

