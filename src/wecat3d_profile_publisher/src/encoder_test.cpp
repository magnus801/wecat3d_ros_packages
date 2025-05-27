#include <iostream>
#include <csignal>
#include <unistd.h>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "EthernetScannerSDK.h"
#include "EthernetScannerSDKDefine.h"
#define WIDTH_X      1280
#define HEIGHT_Z     1024
#define BUFFER_SIZE  WIDTH_X
volatile bool keep_running = true;
void signal_handler(int) {
    keep_running = false;
    std::cout << "\nCtrl+C detected. Shutting down...\n";
}
void send_command(void* handle, const char* cmd) {
    int ret = EthernetScanner_WriteData(handle, (char*)cmd, strlen(cmd));
    std::cout << "Command Sent: " << cmd << " [Return Code: " << ret << "]\n";
}
void read_property(void* handle, const char* name) {
    char buffer[ETHERNETSCANNER_BUFFERSIZEMAX] = {0};
    int ret = EthernetScanner_ReadData(handle, (char*)name, buffer, ETHERNETSCANNER_BUFFERSIZEMAX, 0);
    if (ret == ETHERNETSCANNER_READDATAOK)
        std::cout << "  " << name << " = " << buffer << "\n";
    else
        std::cout << "  Failed to read " << name << " (Code: " << ret << ")\n";
}
int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    auto node = rclcpp::Node::make_shared("wecat3d_runtime_node");
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wecat3d/pointcloud", 10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Connect to sensor
    const char* ip = "192.168.100.1";
    const char* port = "32001";
    void* handle = EthernetScanner_Connect((char*)ip, (char*)port, 1000);
    if (!handle) {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect to sensor at %s:%s", ip, port);
        return -1;
    }
    int status = 0;
    for (int tries = 0; tries < 10 && status != 3; ++tries) {
        usleep(200000);
        EthernetScanner_GetConnectStatus(handle, &status);
    }
    if (status != 3) {
        RCLCPP_ERROR(node->get_logger(), "Sensor not ready. Status: %d", status);
        EthernetScanner_Disconnect(handle);
        return -1;
    }
    // Sensor setup for encoder trigger mode
    send_command(handle, "SetAcquisitionStop\r");
    usleep(500000);
    send_command(handle, "SetTriggerSource=2\r");
    send_command(handle, "SetTriggerEncoderStep=3\r");
    send_command(handle, "SetEncoderTriggerFunction=2\r");
    send_command(handle, "ResetEncoder\r");
    send_command(handle, "SetRangeImageNrProfiles=1\r");
    // Full ROI (no offset, full width and height)
    send_command(handle, "SetROI1OffsetX=0\r");
    send_command(handle, "SetROI1WidthX=1280\r");
    send_command(handle, "SetROI1OffsetZ=0\r");
    send_command(handle, "SetROI1HeightZ=1024\r");
    send_command(handle, "SetROI1StepX=0\r");
    send_command(handle, "SetROI1StepZ=0\r");
    // Auto exposure settings
    send_command(handle, "SetAutoExposureMode=0\r");
    // send_command(handle, "SetHDR=1\r");
    // send_command(handle, "SetExposureTime=750\r");
    // send_command(handle, "SetExposureTime2=1000\r");
    
    // send_command(handle, "SetAutoExposureMode=1\r");
    // send_command(handle, "SetAutoExposureTimeMin=750\r");
    // send_command(handle, "SetAutoExposureTimeMax=20000\r");
    // send_command(handle, "SetAutoExposureIntensityRangeMin=0\r");
    // send_command(handle, "SetAutoExposureIntensityRangeMax=1024\r");
    // send_command(handle, "SetAutoExposureRangeXMin=0\r");
    // send_command(handle, "SetAutoExposureRangeXMax=1278\r");
    // Enable Z and intensity output
    send_command(handle, "SetSignalContentZ=1\r");
    send_command(handle, "SetSignalContentStrength=1\r");
    // Enable internal 3D point calculation
    send_command(handle, "SetInitializeAcquisition\r");
    send_command(handle, "SetLinearizationMode=1\r");
    EthernetScanner_ResetDllFiFo(handle);
    send_command(handle, "SetAcquisitionStart\r");
    int32_t base_enc = 0, last_raw = 0;
    bool initialized = false;
    float total_y_distance = 0.0f;
    RCLCPP_INFO(node->get_logger(), "Acquisition started. Waiting for profiles...");
    while (keep_running && rclcpp::ok()) {
        double xBuf[BUFFER_SIZE], zBuf[BUFFER_SIZE];
        int iBuf[BUFFER_SIZE], wBuf[BUFFER_SIZE];
        unsigned int rawEnc;
        unsigned char usrIO;
        int picCount;
        int n = EthernetScanner_GetXZIExtended(handle, xBuf, zBuf, iBuf, wBuf, BUFFER_SIZE, &rawEnc, &usrIO, 1000, nullptr, 0, &picCount);
        if (n == WIDTH_X) {
            int32_t raw = static_cast<int32_t>(rawEnc);
            if (!initialized) {
                base_enc = last_raw = raw;
                initialized = true;
            }
            int32_t rel_enc = raw - base_enc;
            float y = rel_enc / 1000.0f;  // mm to meters
            total_y_distance = y;
            sensor_msgs::msg::PointCloud2 msg;
            msg.header.stamp = node->now();
            msg.header.frame_id = "map";
            msg.height = 1;
            msg.width = n;
            msg.is_dense = false;
            msg.is_bigendian = false;
            sensor_msgs::PointCloud2Modifier mod(msg);
            mod.setPointCloud2FieldsByString(1, "xyz");
            mod.resize(n);
            sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cur_cloud->width = n;
            cur_cloud->height = 1;
            cur_cloud->is_dense = false;
            cur_cloud->points.resize(n);
            for (int i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
                float x = static_cast<float>(xBuf[i] / 1000.0f);
                float z = static_cast<float>(zBuf[i] / 1000.0f);
                *it_x = x;
                *it_y = y;
                *it_z = z;
                cur_cloud->points[i].x = x;
                cur_cloud->points[i].y = y;
                cur_cloud->points[i].z = z;
            }
            *merged_cloud += *cur_cloud;
            pcl::io::savePCDFileBinaryCompressed("merged_encoder_output.pcd", *merged_cloud);
            pub->publish(msg);
            RCLCPP_INFO(node->get_logger(), "Published %d points | Encoder raw=%d | Y=%.3f m | Total Y=%.3f m", n, raw, y, total_y_distance);
        }
        rclcpp::spin_some(node);
        usleep(500);
    }
    send_command(handle, "SetAcquisitionStop\r");
    usleep(500000);
    EthernetScanner_Disconnect(handle);
    rclcpp::shutdown();
    std::cout << "Sensor disconnected. Shutdown complete.\n";
    return 0;
}
