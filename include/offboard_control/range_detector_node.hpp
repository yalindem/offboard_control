#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


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