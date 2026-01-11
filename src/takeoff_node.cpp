#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;

namespace Offboard
{

    class TakeoffNode : public rclcpp::Node
    {
        public:

            TakeoffNode() : Node("takeoff_node")
            {
                offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
                trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
                vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
                vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", 10, std::bind(&TakeoffNode::vehicleStatusCallback, this, std::placeholders::_1));
                
                timer_ = this->create_wall_timer(50ms, std::bind(&TakeoffNode::timerCallback, this));
            }

        private:

            enum class State
            {
                INIT,
                STREAM_SETPOINTS,
                ARM,
                OFFBOARD,
                TAKEOFF,
                HOVER
            };
            
            void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
            {
                vehicle_status_ = *msg;
            }

            void publishOffboardControlMode()
            {
                px4_msgs::msg::OffboardControlMode msg{};
                msg.position = true;
                msg.velocity = false;
                msg.acceleration = false;
                msg.attitude = false;
                msg.body_rate = false;
                msg.timestamp = now().nanoseconds() / 1000;
                offboard_control_mode_pub_->publish(msg);
            }

            void publishTrajectorySetpoint(float z)
            {
                px4_msgs::msg::TrajectorySetpoint msg{};
                msg.position = {0.0f, 0.0f, z};   // NED: z negative = up
                msg.yaw = 0.0f;
                msg.timestamp = now().nanoseconds() / 1000;
                trajectory_setpoint_pub_->publish(msg);
            }

            void sendVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
            {
                px4_msgs::msg::VehicleCommand msg{};
                msg.command = command;
                msg.param1 = param1;
                msg.param2 = param2;
                msg.target_system = 1;
                msg.target_component = 1;
                msg.source_system = 1;
                msg.source_component = 1;
                msg.from_external = true;
                msg.timestamp = now().nanoseconds() / 1000;
                vehicle_command_pub_->publish(msg);
            }

            void arm()
            {
                sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                RCLCPP_INFO(this->get_logger(), "Arm command sent");
            }

            void disarm()
            {
                sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
                RCLCPP_INFO(this->get_logger(), "Disarm command sent");
            }

            void setOffboardMode()
            {
                sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
            }

            void timerCallback()
            {
                publishOffboardControlMode();

                switch(state_)
                {
                    case State::INIT:
                        state_ = State::STREAM_SETPOINTS;
                        break;

                    case State::STREAM_SETPOINTS:
                        publishTrajectorySetpoint(0.0f);
                        if (++offboard_counter_ > 20)
                            state_ = State::ARM;
                        break;
                    
                    case State::ARM:
                        arm();
                        state_ = State::OFFBOARD;
                        break;

                    case State::OFFBOARD:
                        setOffboardMode();
                        state_ = State::TAKEOFF;
                        break;

                    case State::TAKEOFF:
                        publishTrajectorySetpoint(-2.0f); // 2m takeoff
                        state_ = State::HOVER;
                        break;

                    case State::HOVER:
                        publishTrajectorySetpoint(-2.0f);
                        break;
                }
            }

            State state_{State::INIT};
            uint64_t offboard_counter_{0};
            px4_msgs::msg::VehicleStatus vehicle_status_{};

            rclcpp::TimerBase::SharedPtr timer_;
            
            rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
            rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
            rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
            rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
            
    };  
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Offboard::TakeoffNode>());
    return 0;
}
