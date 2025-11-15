#include "offboard_control/offboard_control_node.hpp"


using namespace std::chrono_literals;

constexpr double BARO_CONST = 29.27;

namespace px4_offboard
{
    OffboardController::OffboardController(std::string node_name) : Node(node_name)
    {
        rclcpp::QoS qos_profile = rclcpp::QoS(1).best_effort(); 
        vehicle_command_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", 
            qos_profile,
            std::bind(&OffboardController::vehicle_status_callback, this, std::placeholders::_1));

        vehicle_sensor_baro_sub_ = this->create_subscription<VehicleSensorBarometerMessage>(
            "/fmu/out/sensor_baro", 
            qos_profile,
            std::bind(&OffboardController::vehicle_sensor_barometer_callback, this, std::placeholders::_1));

        vehicle_sensor_imu_sub_ = this->create_subscription<SensorCombinedMessage>(
            "/fmu/out/sensor_combined", 
            qos_profile,
            std::bind(&OffboardController::sensor_combined_callback, this, std::placeholders::_1));
        
        this->vehicle_command_client_ = this->create_client<VehicleCommandSrv>("/fmu/vehicle_command");

        rclcpp::Rate loop_rate(3.0);
        loop_rate.sleep(); 

        if (!vehicle_command_client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "Service not available");
        }
        else{
            RCLCPP_INFO(this->get_logger(), GREEN("Service is ready"));
        }

        trajectory_pub_ = this->create_publisher<TrajectorySetpointMessage>("/fmu/in/trajectory_setpoint", 10);
        offboard_mode_pub_ = this->create_publisher<OffboardControlModeMessage>("fmu/in/offboard_control_mode", 10);

        timer_ = this->create_wall_timer(500ms, std::bind(&OffboardController::publish_offboard_control_mode, this));
    }

    void OffboardController::switch_to_offboard_mode(){
        RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
        request_vehicle_command(VehicleCommandMessage::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void OffboardController::publish_trajectory_setpoint(float x, float y, float z, float yaw)
    {
        TrajectorySetpointMessage msg{};
        msg.position = {x, y, z};
        msg.yaw = yaw;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_pub_->publish(msg);
    }


    void OffboardController::publish_offboard_control_mode()
    {
        OffboardControlModeMessage msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_mode_pub_->publish(msg);

        this->run();
    }


    void OffboardController::arm()
    {
        RCLCPP_INFO(this->get_logger(), "Requesting arm");
        request_vehicle_command(VehicleCommandMessage::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        this->arming_requested_ = true;
    }

    float OffboardController::calculate_barometric_height(const float pressure, const float temp)
    {
        float T = (initial_temp_ + temp) / 2.0;
        return ((287.05 / 9.80665)  * T * std::log(this->initial_pressure_ / pressure));
    }

    void OffboardController::vehicle_sensor_barometer_callback(const VehicleSensorBarometerMessage::SharedPtr msg)
    {
        if (!this->is_baro_ready_)
        {
            this->initial_pressure_ = msg->pressure*0.01;
            this->initial_temp_ = msg->temperature;
            this->is_baro_ready_ = true;
        }

        barometric_height_ = calculate_barometric_height(msg->pressure*0.01, msg->temperature);
        RCLCPP_INFO(this->get_logger(), "barometric_height: %f", barometric_height_);
    }

    void OffboardController::sensor_combined_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
    {

    }

    void OffboardController::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        this->pre_flight_checks_pass_ = msg->pre_flight_checks_pass;

        if(this->is_baro_ready_ == false)
        {
            this->pre_flight_checks_pass_ = false;
        }

        switch (msg->arming_state)
        {
            case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
                this->state_ = State::armed;
                break;
            case px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED:
                this->state_ = State::disarmed;
                break;
            default:
                this->state_ = State::pre_flight;
                break;
        }

        switch (msg->nav_state)
        {
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:
                nav_state_ = NavState::manual;
                break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:
                nav_state_ = NavState::altitude;
                break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
                nav_state_ = NavState::position;
                break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
                nav_state_ = NavState::offboard;
                break;
            default:
                nav_state_ = NavState::manual;
                break;

        }
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

        auto result = vehicle_command_client_->async_send_request(request, 
                                                                  std::bind(&OffboardController::response_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Command send");
    }
	
    void OffboardController::response_callback(VehicleCommandSharedFuture future)
    {
        auto response = future.get();
        switch (response->reply.result)
        {
            case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
                RCLCPP_INFO(this->get_logger(), GREEN("Command accepted by PX4"));
                RCLCPP_INFO(this->get_logger(), "Command: %u", response->reply.command);
                break;

            case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                RCLCPP_WARN(this->get_logger(), "Command temporarily rejected");
                break;
            case px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
                RCLCPP_WARN(this->get_logger(), "Command denied");
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "Command result: %u", response->reply);
                break;
        }


    }

    void OffboardController::run()
    {
        if(this->pre_flight_checks_pass_)
        {  
            if(this->nav_state_ != NavState::offboard)
            {
                RCLCPP_WARN(this->get_logger(), "Navigation state: %d", this->nav_state_);
                this->switch_to_offboard_mode();
            }

            else if(this->nav_state_ == NavState::offboard && this->state_ != State::armed && this->is_armed_ == false)
            {
                RCLCPP_WARN(this->get_logger(), "State: %d", this->state_);
                this->arm();
                this->publish_trajectory_setpoint(0.0, 0.0, 0.0, -3.14); 
            }

            else if(this->state_ == State::armed && this->is_armed_ == false)
            {
                RCLCPP_INFO(this->get_logger(), "Armed");
                this->is_armed_ = true;
            }

            if(this->state_ == State::armed && this->is_armed_)
            {
                this->publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14); 
            }

        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Pre flight check not passed");
        }

        
        
    }
    

}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<px4_offboard::OffboardController>("offboard_control");
    rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}