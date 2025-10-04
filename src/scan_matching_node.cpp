#include "offboard_control/scan_matching_node.hpp"


namespace px4_offboard
{

    ScanMatchingNode::ScanMatchingNode(std::string node_name) : Node(node_name)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10, std::bind(&ScanMatchingNode::point_cloud_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void ScanMatchingNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // PointCloud2 mesajını PCL formatına dönüştür
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Voxel grid filtrelemesi ile veri boyutunu küçült
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.filter(*cloud);

        // ICP algoritmasını başlat
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        // Burada hedef point cloud'u (örneğin, önceki bir tarama) belirtmelisiniz
        // icp.setInputTarget(previous_cloud);

        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud);

        if (icp.hasConverged())
        {
            RCLCPP_INFO(this->get_logger(), "ICP converged with score: %f", icp.getFitnessScore());
            // Burada dönüşüm bilgilerini kullanarak robotun pozunu güncelleyebilirsiniz
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }
    }

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_offboard::ScanMatchingNode>("ScanMatchingNode"));
    rclcpp::shutdown();
    return 0;
}