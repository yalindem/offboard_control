#include "offboard_control/target_localizer_node.hpp"

namespace Drone::Localizer
{

    TargetLocalizer::TargetLocalizer() : Node("target_localizer")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TargetLocalizer::transform_target, this));
    }

    void TargetLocalizer::transform_target()
    {
        geometry_msgs::msg::PointStamped point_in_cam;
        point_in_cam.header.frame_id = "flow_link";
        point_in_cam.header.stamp = this->get_clock()->now();
        point_in_cam.point.x = 0.0;
        point_in_cam.point.y = 0.0;
        point_in_cam.point.z = 3.0;

        geometry_msgs::msg::PointStamped point_in_odom;

        try
        {
            auto transform = tf_buffer_->lookupTransform(
                "odom", "camera_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Dönüşüm hatası: %s", ex.what());
        }
        
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