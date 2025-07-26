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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);  
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*cloud_filtered); 

        RCLCPP_INFO(this->get_logger(), "Received point cloud with %lu points", cloud_filtered->points.size());

        for (size_t i = 0; i < std::min<size_t>(5, cloud_filtered->points.size()); ++i) {
            const auto& pt = cloud_filtered->points[i];
            float distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            RCLCPP_INFO(this->get_logger(), "Point: (%.2f, %.2f, %.2f), Distance: %.2f m", pt.x, pt.y, pt.z, distance);
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