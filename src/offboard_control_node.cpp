#include "offboard_control/offboard_control_node.hpp"


using namespace std::chrono_literals;

namespace px4_offboard
{
    OffboardController::OffboardController(std::string node_name) : Node(node_name)
    {
        this->vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
    }

    void OffboardController::arm()
    {
        RCLCPP_INFO(this->get_logger(), "requesting arm");
        request_vehicle_command(VehicleCommandMessage::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }

    void OffboardController::request_vehicle_command(uint16_t command, float param1, float param2)
    {
        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

        VehicleCommandMessage msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        request->request = msg;

        service_done_ = false;
        auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                                std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Command send");
    }
	
    void OffboardController::response_callback(VehicleCommandFuture future)
    {

    }

}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<px4_offboard::OffboardController>("px4_offboard/offboard_control");

	rclcpp::shutdown();
	return 0;
}