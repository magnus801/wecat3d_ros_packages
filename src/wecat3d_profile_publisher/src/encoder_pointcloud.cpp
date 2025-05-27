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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    auto node = rclcpp::Node::make_shared("wecat3d_runtime_node");
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wecat3d/pointcloud", 10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

    send_command(handle, "SetAcquisitionStop\r");
    usleep(500000);
    send_command(handle, "SetTriggerSource=2\r");
    send_command(handle, "SetTriggerEncoderStep=3\r");
    send_command(handle, "SetEncoderTriggerFunction=2\r");
    send_command(handle, "ResetEncoder\r");
    send_command(handle, "SetRangeImageNrProfiles=1\r");
    send_command(handle, "SetROI1OffsetX=0\r");
    send_command(handle, "SetROI1WidthX=1280\r");
    send_command(handle, "SetROI1OffsetZ=0\r");
    send_command(handle, "SetROI1HeightZ=1024\r");
    send_command(handle, "SetROI1StepX=0\r");
    send_command(handle, "SetROI1StepZ=0\r");
    send_command(handle, "SetAutoExposureMode=0\r");
    send_command(handle, "SetHDR=1\r");
    send_command(handle, "SetExposureTime=150\r");
    send_command(handle, "SetExposureTime2=10000\r");
    send_command(handle, "SetSignalContentZ=1\r");
    send_command(handle, "SetSignalContentStrength=1\r");
    send_command(handle, "SetInitializeAcquisition\r");
    send_command(handle, "SetLinearizationMode=1\r");

    EthernetScanner_ResetDllFiFo(handle);
    send_command(handle, "SetAcquisitionStart\r");

    int32_t base_enc = 0;
    bool initialized = false;
    float total_y_distance = 0.0f;
    RCLCPP_INFO(node->get_logger(), "Acquisition started. Waiting for HDR profiles...");

    while (keep_running && rclcpp::ok()) {
        double xBuf1[BUFFER_SIZE], zBuf1[BUFFER_SIZE];
        int iBuf1[BUFFER_SIZE], wBuf1[BUFFER_SIZE];
        unsigned int rawEnc1;
        unsigned char usrIO1;
        int picCount1;

        double xBuf2[BUFFER_SIZE], zBuf2[BUFFER_SIZE];
        int iBuf2[BUFFER_SIZE], wBuf2[BUFFER_SIZE];
        unsigned int rawEnc2;
        unsigned char usrIO2;
        int picCount2;

        int n1 = EthernetScanner_GetXZIExtended(handle, xBuf1, zBuf1, iBuf1, wBuf1, BUFFER_SIZE, &rawEnc1, &usrIO1, 1000, nullptr, 0, &picCount1);
        if (n1 != WIDTH_X) continue;

        int n2 = EthernetScanner_GetXZIExtended(handle, xBuf2, zBuf2, iBuf2, wBuf2, BUFFER_SIZE, &rawEnc2, &usrIO2, 1000, nullptr, 0, &picCount2);
        if (n2 != WIDTH_X) continue;

        int32_t raw = static_cast<int32_t>(rawEnc1);
        if (!initialized) {
            base_enc = raw;
            initialized = true;
        }
        float y = (raw - base_enc) / 1000.0f;
        total_y_distance = y;

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = node->now();
        msg.header.frame_id = "map";
        msg.height = 1;
        msg.width = n1;
        msg.is_dense = false;
        msg.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(n1);
        sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cur_cloud->width = n1;
        cur_cloud->height = 1;
        cur_cloud->is_dense = false;
        cur_cloud->points.resize(n1);

        for (int i = 0; i < n1; ++i, ++it_x, ++it_y, ++it_z) {
            int intensity1 = iBuf1[i];
            int intensity2 = iBuf2[i];  
            float x, z;

            if (intensity1 >= intensity2) {
                x = static_cast<float>(xBuf1[i] / 1000.0f);
                z = static_cast<float>(zBuf1[i] / 1000.0f);
            } else {
                x = static_cast<float>(xBuf2[i] / 1000.0f);
                z = static_cast<float>(zBuf2[i] / 1000.0f);
            }

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
        RCLCPP_INFO(node->get_logger(), "Published %d points | Y=%.3f m | Total Y=%.3f m", n1, y, total_y_distance);

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
