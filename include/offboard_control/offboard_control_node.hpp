#ifndef CPP_OFFBOARD_CONTROLLER
#define CPP_OFFBOARD_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include "px4_msgs/srv/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"

using VehicleCommandSrv = px4_msgs::srv::VehicleCommand;
using VehicleCommandClient = rclcpp::Client<VehicleCommandSrv>;
using VehicleCommandSharedPtr = VehicleCommandClient::SharedPtr;
using VehicleCommandSharedFuture = VehicleCommandClient::SharedFuture;
using VehicleCommandMessage = px4_msgs::msg::VehicleCommand;
using VehicleCommandMessageAck = px4_msgs::msg::VehicleCommandAck;
using VehicleStatusMessage = px4_msgs::msg::VehicleStatus;
using VehicleCommandMessageSubscriber = rclcpp::Subscription<VehicleStatusMessage>::SharedPtr;
using TrajectorySetpointMessage = px4_msgs::msg::TrajectorySetpoint;
using TrajectorySetpointPublisher = rclcpp::Publisher<TrajectorySetpointMessage>::SharedPtr;
using OffboardControlModeMessage = px4_msgs::msg::OffboardControlMode;
using OffboardControlModePublisher = rclcpp::Publisher<OffboardControlModeMessage>::SharedPtr;

enum State {
    pre_flight = 0,
    armed = 1,    
    on_air = 2,
    disarmed = 3
};

enum NavState {
    manual = 0,
    altitude = 1,
    position = 2,
    offboard = 14
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
            void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
            void switch_to_offboard_mode();
            void publish_trajectory_setpoint(float x, float y, float z, float yaw);
            void publish_offboard_control_mode();

            VehicleCommandMessageSubscriber vehicle_command_sub_;
            bool arming_requested_{false};

            State state_ {State::pre_flight};
            NavState nav_state_ {NavState::manual};

            TrajectorySetpointPublisher trajectory_pub_;
            OffboardControlModePublisher offboard_mode_pub_;

            rclcpp::TimerBase::SharedPtr timer_;

            bool pre_flight_checks_pass_{false};
    };

}

#endif