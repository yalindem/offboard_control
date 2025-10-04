#include "offboard_control/offboard_control_node.hpp"


using namespace std::chrono_literals;

namespace px4_offboard
{
    OffboardController::OffboardController(std::string node_name) : Node(node_name)
    {
        this->vehicle_command_client_ = this->create_client<VehicleCommandSrv>("/fmu/vehicle_command");
        if (!vehicle_command_client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "Service not available");
        }
        else{
            RCLCPP_INFO(this->get_logger(), GREEN("Service is ready"));
        }
    }

    void OffboardController::arm()
    {
        RCLCPP_INFO(this->get_logger(), "Requesting arm");
        request_vehicle_command(VehicleCommandMessage::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
    }

    void OffboardController::request_vehicle_command(uint16_t command, float param1, float param2)
    {
        auto request = std::make_shared<VehicleCommandSrv::Request>();

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
        auto result = vehicle_command_client_->async_send_request(request, 
                                                                  std::bind(&OffboardController::response_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Command send");
    }
	
    void OffboardController::response_callback(VehicleCommandSharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Response callback Send starting ...");
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "VehicleCommand response received!");

        switch (response->reply.result)
        {
            case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
                RCLCPP_INFO(this->get_logger(), GREEN("Command accepted by PX4"));
                RCLCPP_INFO(this->get_logger(), "Command: %u", response->reply.command);
                service_done_ = true;
                arm_retry_count_ = 0;
                break;

            case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
                RCLCPP_WARN(this->get_logger(), "Command temporarily rejected");
                if (arm_retry_count_ < max_arm_retries_)
                {
                    arm_retry_count_++;
                    RCLCPP_INFO(this->get_logger(), "Retry attempt %d", arm_retry_count_);
                    this->arm();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Max ARM retries reached");
                    service_done_ = true;
                }
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "Command result: %u", response->reply);
                arm_retry_count_ = 0;
                service_done_ = true;
                break;
        }


    }

    void OffboardController::run()
    {
        RCLCPP_INFO(this->get_logger(), "Trying to Arm");
        this->arm();
    }

}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<px4_offboard::OffboardController>("offboard_control");
    node->run();
    rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}