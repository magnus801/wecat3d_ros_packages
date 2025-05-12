#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "EthernetScannerSDK.h"

#define BUFFER_SIZE 3200

class Wecat3DCloudPublisher : public rclcpp::Node {
public:
    Wecat3DCloudPublisher()
    : Node("wecat3d_pointcloud_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("wecat3d/pointcloud", 10);

        handle_ = EthernetScanner_Connect((char*)"192.168.100.1", (char*)"32001", 0);
        if (!handle_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to sensor.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to weCat3D sensor.");

        // Optional sensor setup
        sendCommand("SetTriggerSource=2\r");
        sendCommand("SetTriggerEncoderStep=10\r");
        sendCommand("SetEncoderTriggerFunction=1\r");
        sendCommand("ResetEncoder\r\n");
        sendCommand("StartAcquisition\r\n");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Wecat3DCloudPublisher::grabAndPublish, this)
        );
    }

    ~Wecat3DCloudPublisher() {
        if (handle_) {
            EthernetScanner_Disconnect(handle_);
            RCLCPP_INFO(this->get_logger(), "Disconnected from sensor.");
        }
    }

private:
    void sendCommand(const char* command) {
        int res = EthernetScanner_WriteData(handle_, (char*)command, strlen(command));
        if (res != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to send command: %s", command);
        }
    }

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
            NULL,
            0,
            &pictureCount
        );

        if (numPoints <= 0) {
            RCLCPP_WARN(this->get_logger(), "No points received.");
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = now();
        cloud_msg.header.frame_id = "laser_frame";
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

        for (int i = 0; i < numPoints; ++i, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = xBuffer[i] / 1000.0f;  // mm to meters
            *iter_y = static_cast<float>(encoder) / 1000.0f;  // mm to meters
            *iter_z = zBuffer[i] / 1000.0f;
        }

        publisher_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published %d points at encoder Y: %d", numPoints, encoder);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void* handle_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Wecat3DCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

