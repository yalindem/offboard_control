#include "offboard_control/offboard_control_node.hpp"


using namespace std::chrono_literals;

namespace px4_offboard
{
    OffboardControllerNode::OffboardControllerNode(std::string node_name) : Node(node_name)
    {
        //offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        //trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1",
            rclcpp::SensorDataQoS(),
            std::bind(&OffboardControllerNode::vehicle_status_callback, this, std::placeholders::_1)
        );
        

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControllerNode::timer_callback, this)); // 2hz is important because px4 requires to be able to be in offboard mode

    }

    void OffboardControllerNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        if((int)(msg->arming_state) == (int)(px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED))
        {
            this->is_armed_ = true;
        }
    }

    void OffboardControllerNode::publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {2.0, 0.0, -3.0};
        msg.yaw = 0;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void OffboardControllerNode::timer_callback()
    {
        if(!this->is_armed_)
        {
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
        }
        else
        {
            if (!offboard_enabled_)
            {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                offboard_enabled_ = true;
            }
            this->publish_offboard_control_mode();
            this->publish_trajectory_setpoint();
        }
        
    }


    void OffboardControllerNode::publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
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
	auto node = std::make_shared<px4_offboard::OffboardControllerNode>("offboard_control");
    rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}