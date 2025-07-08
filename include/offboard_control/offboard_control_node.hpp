#ifndef CPP_OFFBOARD_CONTROLLER_NODE_HPP_
#define CPP_OFFBOARD_CONTROLLER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

namespace px4_offboard
{
    class OffboardControllerNode : public rclcpp::Node
    {
        public:
            OffboardControllerNode(std::string node_name);
    
        private:
            void publish_offboard_control_mode();
            void publish_trajectory_setpoint();
	        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
            void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
            void arm();
            void takeoff();
            void timer_callback();

            rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
            rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
            //rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
            rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

            bool is_armed_ {false};
            bool offboard_enabled_{false};
    };
}


#endif