#ifndef CPP_OFFBOARD_CONTROLLER
#define CPP_OFFBOARD_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "px4_msgs/srv/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using VehicleCommandSrv = px4_msgs::srv::VehicleCommand;
using VehicleCommandClient = rclcpp::Client<VehicleCommandSrv>;
using VehicleCommandSharedPtr = VehicleCommandClient::SharedPtr;
using VehicleCommandSharedFuture = VehicleCommandClient::SharedFuture;
using VehicleCommandMessage = px4_msgs::msg::VehicleCommand;
using VehicleCommandMessageAck = px4_msgs::msg::VehicleCommandAck;
using VehicleStatusMessage = px4_msgs::msg::VehicleStatus;
using VehicleCommandMessageSubscriber = rclcpp::Subscription<VehicleStatusMessage>::SharedPtr;

enum State {
    pre_flight = 0,
    armed,    
    on_air,
    disarmed 
};

#define GREEN(text) "\033[1;32m" text "\033[0m"

namespace px4_offboard
{

    class OffboardController : public rclcpp::Node
    {
        public:
            OffboardController(std::string node_name);
            void run();

        private:
            VehicleCommandSharedPtr vehicle_command_client_;
            void arm();
            void request_vehicle_command(std::uint16_t command, float param1 = 0.0, float param2 = 0.0);
	        void response_callback(VehicleCommandSharedFuture future);
            void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);


            VehicleCommandMessageSubscriber vehicle_command_sub_;

            bool service_done_ = false;
            int arm_retry_count_ = 0;
            const int max_arm_retries_ = 5;
            bool arming_requested_{false};

            State state_ {State::pre_flight};
    };

}

#endif