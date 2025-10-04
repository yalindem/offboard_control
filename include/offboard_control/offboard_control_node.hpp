#ifndef CPP_OFFBOARD_CONTROLLER
#define CPP_OFFBOARD_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "px4_msgs/srv/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

using VehicleCommandClient = rclcpp::Client<px4_msgs::srv::VehicleCommand>;
using VehicleCommandSharedPtr = VehicleCommandClient::SharedPtr;
using VehicleCommandFuture = VehicleCommandClient::Future;
using VehicleCommandMessage = px4_msgs::msg::VehicleCommand;
using VehicleCommandMessageAck = px4_msgs::msg::VehicleCommandAck;

namespace px4_offboard
{

    class OffboardController : public rclcpp::Node
    {
        public:
            OffboardController(std::string node_name);

        private:
            VehicleCommandSharedPtr vehicle_command_client_;
            void arm();
            void request_vehicle_command(std::uint16_t command, float param1, float param2);
	        void response_callback(VehicleCommandFuture future);
    };

}

#endif