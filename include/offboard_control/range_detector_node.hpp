#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cmath>


namespace px4_offboard
{
    class PointCloudSubscriber : public rclcpp::Node
    {
    public:
        PointCloudSubscriber(std::string node_name);

    private:
        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    };
}