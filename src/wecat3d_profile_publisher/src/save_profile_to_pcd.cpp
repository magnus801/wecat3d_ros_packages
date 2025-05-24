#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "EthernetScannerSDK.h"

#include <iostream>
#include <string>
#include <unistd.h>
#include <csignal>
#include <cmath>
#include <cstring>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include <sstream>

#define BUFFER_SIZE 3200
#define MAX_Y_JUMP 100  // mm

class Wecat3DCloudPublisher : public rclcpp::Node {
public:
    Wecat3DCloudPublisher()
    : Node("wecat3d_profile_publisher_with_filter"),
      last_encoder_(-1),
      scan_count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("wecat3d/pointcloud", 10);

        // Connect to sensor
        handle_ = EthernetScanner_Connect((char*)"192.168.100.1", (char*)"32001", 0);
        if (!handle_) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed.");
            rclcpp::shutdown();
            return;
        }

        int status = 0, retries = 0;
        while (status != 3 && retries++ < 10) {
            EthernetScanner_GetConnectStatus(handle_, &status);
            usleep(200000);
        }
        if (status != 3) {
            RCLCPP_ERROR(this->get_logger(), "Failed to fully connect. Status: %d", status);
            EthernetScanner_Disconnect(handle_);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to sensor.");

        int resetStatus = EthernetScanner_ResetDllFiFo(handle_);
        if (resetStatus == 0) {
            RCLCPP_INFO(this->get_logger(), "DLL FIFO buffer reset.");
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Wecat3DCloudPublisher::grabAndPublish, this)
        );
    }

    ~Wecat3DCloudPublisher() {
        if (handle_) {
            EthernetScanner_Disconnect(handle_);
            RCLCPP_INFO(this->get_logger(), "Disconnected.");
        }
    }

private:
    void grabAndPublish() {
        double xBuffer[BUFFER_SIZE] = {0};
        double zBuffer[BUFFER_SIZE] = {0};
        int intensityBuffer[BUFFER_SIZE] = {0};
        int signalWidthBuffer[BUFFER_SIZE] = {0};
        int32_t encoder = 0;
        unsigned char usrIO = 0;
        int pictureCount = 0;

        int numPoints = EthernetScanner_GetXZIExtended(
            handle_,
            xBuffer,
            zBuffer,
            intensityBuffer,
            signalWidthBuffer,
            BUFFER_SIZE,
            (unsigned int*)&encoder,
            &usrIO,
            1000,
            nullptr,
            0,
            &pictureCount
        );

        if (numPoints <= 0) return;

        int fifoUsage = EthernetScanner_GetDllFiFoState(handle_);
        if (fifoUsage >= 90) {
            RCLCPP_WARN(this->get_logger(), "FIFO usage high: %d%%", fifoUsage);
        }

        if (last_encoder_ != -1 && std::abs(encoder - last_encoder_) > MAX_Y_JUMP) {
            RCLCPP_WARN(this->get_logger(), "Skipped profile due to encoder jump: ΔY = %d", (encoder - last_encoder_));
            last_encoder_ = encoder;
            return;
        }
        last_encoder_ = encoder;

        // Create ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser_frame";
        msg.height = 1;
        msg.width = numPoints;
        msg.is_dense = false;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(numPoints);

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        // Create PCL cloud for saving
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = numPoints;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(numPoints);

        for (int i = 0; i < numPoints; ++i, ++iter_x, ++iter_y, ++iter_z) {
            float x = xBuffer[i] / 1000.0f;
            float y = static_cast<float>(encoder) / 1000.0f;
            float z = zBuffer[i] / 1000.0f;

            *iter_x = x;
            *iter_y = y;
            *iter_z = z;

            cloud->points[i].x = x;
            cloud->points[i].y = y;
            cloud->points[i].z = z;
        }

        // Save PCD file with incremental name
        std::ostringstream filename;
        filename << "scan_" << std::setw(4) << std::setfill('0') << scan_count_++ << ".pcd";
        pcl::io::savePCDFileASCII(filename.str(), *cloud);
        RCLCPP_INFO(this->get_logger(), "Saved to %s", filename.str().c_str());

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published profile: %d pts | Encoder Y: %d", numPoints, encoder);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void* handle_;
    int32_t last_encoder_;
    int scan_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Wecat3DCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
