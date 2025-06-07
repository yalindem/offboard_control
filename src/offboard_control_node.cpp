#include "offboard_control/offboard_control_node.hpp"


using namespace std::chrono_literals;

namespace px4_offboard
{
    OffboardControllerNode::OffboardControllerNode(std::string node_name) : Node(node_name)
    {
        //offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        //trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControllerNode::timer_callback, this)); // 2hz is important because px4 requires to be able to be in offboard mode
        offboard_setpoint_counter_ = 0;

    }

    void OffboardControllerNode::timer_callback()
    {
        this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    }

    void OffboardControllerNode::publish_vehicle_command(uint16_t command, float param1, float param2)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
    

}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<px4_offboard::OffboardControllerNode>("px4_offboard/offboard_control");

	rclcpp::shutdown();
	return 0;
}