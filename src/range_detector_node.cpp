#include "offboard_control/range_detector_node.hpp"

namespace px4_offboard
{
    PointCloudSubscriber::PointCloudSubscriber(std::string node_name) : Node(node_name)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/world/walls/model/x500_mono_cam_0/link/lidar_link/sensor/front_lidar/scan/points", 10,
            std::bind(&PointCloudSubscriber::pointcloud_callback, this, std::placeholders::_1));
    }

    void PointCloudSubscriber::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        for(size_t i = 0; i < msg->height * msg->width; ++i, ++iter_x, ++iter_y, ++iter_z) 
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            float distance = std::sqrt(x * x + y * y + z * z);
            RCLCPP_INFO(this->get_logger(), "Point: (%.2f, %.2f, %.2f), Distance: %.2f m", x, y, z, distance);
        }

    }
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_offboard::PointCloudSubscriber>("pointcloud_subscriber"));
    rclcpp::shutdown();
    return 0;
}