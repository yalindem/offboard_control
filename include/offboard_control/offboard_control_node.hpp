#ifndef CPP_OFFBOARD_CONTROLLER
#define CPP_OFFBOARD_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <px4_msgs/srv/vehicle_command.hpp>


namespace px4_offboard
{

    class OffboardController : public rclcpp::Node
    {
        public:
            OffboardController(std::string node_name);

        private:
            rclcpp::Subscription<px4_msgs::srv::VehicleCommand>::SharedPtr sub_vehicle_command_;
    };

}

#endif