#include "offboard_control/target_localizer_node.hpp"

namespace Drone::Localizer
{

    TargetLocalizer::TargetLocalizer() : Node("target_localizer")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TargetLocalizer::transform_target, this));
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&TargetLocalizer::odom_callback, this, std::placeholders::_1));
    }

    void TargetLocalizer::transform_target()
    {
        
        
    }

    void TargetLocalizer::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg->position[0];
        t.transform.translation.y = msg->position[1];
        t.transform.translation.z = msg->position[2];

        t.transform.rotation.x = msg->q[1];
        t.transform.rotation.y = msg->q[2];
        t.transform.rotation.z = msg->q[3];
        t.transform.rotation.w = msg->q[0];
        
        tf_broadcaster_->sendTransform(t);
    }
    
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Drone::Localizer::TargetLocalizer>());
  rclcpp::shutdown();
  return 0;
}