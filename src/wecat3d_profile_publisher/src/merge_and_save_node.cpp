#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class PointCloudMerger : public rclcpp::Node {
public:
    PointCloudMerger()
    : Node("pointcloud_merger_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wecat3d/pointcloud", 10,
            std::bind(&PointCloudMerger::callback, this, std::placeholders::_1)
        );

        merged_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        RCLCPP_INFO(this->get_logger(), "Subscribed to /wecat3d/pointcloud and merging...");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> current;
        pcl::fromROSMsg(*msg, current);

        *merged_cloud_ += current;

        // Save merged result
        pcl::io::savePCDFileASCII("merged.pcd", *merged_cloud_);
        RCLCPP_INFO(this->get_logger(), "Appended %lu points (total: %lu)", current.size(), merged_cloud_->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

