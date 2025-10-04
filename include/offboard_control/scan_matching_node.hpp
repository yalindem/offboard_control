#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_ros/point_cloud.hpp"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/filters/voxel_grid.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"



namespace px4_offboard
{
    class ScanMatchingNode : public rclcpp::Node
    {
        public:
            ScanMatchingNode(std::string node_name);

        private:
            void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}

