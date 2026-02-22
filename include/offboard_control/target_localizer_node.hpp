#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


namespace Drone::Localizer
{
    class TargetLocalizer : public rclcpp::Node{
        public:
            TargetLocalizer();

            void transform_target();
            void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

        private:
            std::shared_ptr<rclcpp::TimerBase> timer_{nullptr};
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            //rclcpp::TimerBase::SharedPtr timer_{nullptr};
            //tf2_ros::TransformListener::SharedPtr tf_listener_{nullptr};
            

    };
}