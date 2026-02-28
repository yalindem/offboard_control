#include "offboard_control/offboard_control_node.hpp"


using namespace std::chrono_literals;

constexpr double BARO_CONST = 29.27;
constexpr float GRAVITY_Z_ = -9.80665;

namespace Drone::px4_offboard
{
    OffboardController::OffboardController(std::string node_name) : Node(node_name)
    {
        //height_estimator_ = std::make_unique<Estimator::HeightEstimator>();

        rclcpp::QoS qos_profile = rclcpp::QoS(1).best_effort(); 
        vehicle_command_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", 
            qos_profile,
            std::bind(&OffboardController::vehicle_status_callback, this, std::placeholders::_1));
        

        local_pos_sub_ = this->create_subscription<VehicleLocalPositionMessage>(
            "/fmu/out/vehicle_local_position_v1", qos_profile,
            std::bind(&OffboardController::local_position_callback, this, std::placeholders::_1));

        baro_filter_ = std::make_shared<Drone::px4_offboard::FastMedianFilter<15>>();

        vehicle_sensor_baro_sub_ = this->create_subscription<VehicleSensorBarometerMessage>(
            "/fmu/out/sensor_baro", 
            qos_profile,
            std::bind(&OffboardController::vehicle_sensor_barometer_callback, this, std::placeholders::_1));
        
        vehicle_sensor_imu_sub_ = this->create_subscription<SensorCombinedMessage>(
            "/fmu/out/sensor_combined", 
            qos_profile,
            std::bind(&OffboardController::sensor_combined_callback, this, std::placeholders::_1));
        /*
        vehicle_sensor_gps_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>(
            "/fmu/out/vehicle_gps_position", 
            qos_profile,
            std::bind(&OffboardController::vehicle_sensor_gps_callback, this, std::placeholders::_1));
        
        vehicle_attitude_sub_ = this->create_subscription<VehicleAttitudeMessage>(
            "/fmu/out/vehicle_attitude", 
            qos_profile, 
            std::bind(&OffboardController::attitude_callback, this, std::placeholders::_1));
        */
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

        this->create_waypoints();
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
        float T_kelvin = ((initial_temp_ + temp) / 2.0f) + 273.15f;
        float gas_constant_factor = 287.05f / 9.80665f;
        return (gas_constant_factor * T_kelvin * std::log(this->initial_pressure_ / pressure));
    }

    void OffboardController::vehicle_sensor_gps_callback(const VehicleSensorGPSMessage::SharedPtr msg)
    {
        /*
        std::cout << "latitude_deg: " << msg->latitude_deg  << std::endl;
        std::cout << "longitude_deg: " << msg->longitude_deg  << std::endl;
        std::cout << "altitude_msl_m: " << msg->altitude_msl_m  << std::endl;
        */
    }

    void OffboardController::vehicle_sensor_barometer_callback(const VehicleSensorBarometerMessage::SharedPtr msg)
    {
        float current_p = msg->pressure * 0.01f; // hPa cinsinden

        // Kalibrasyon (ilk çalıştırma)
        if (!this->is_baro_ready_)
        {
            this->initial_pressure_ = current_p;
            this->initial_temp_ = msg->temperature;
            this->is_baro_ready_ = true;
            this->last_filtered_height_ = 0.0f;  // Başlangıç değeri
            
            RCLCPP_INFO(this->get_logger(), "Barometre Sıfırlandı: %.2f hPa", initial_pressure_);
            return; 
        }
        float raw_height = calculate_barometric_height(current_p, msg->temperature);
        if(baro_filter_->isFilled())
        {
            baro_filter_->reset();
        }
        float median_height = baro_filter_->update(raw_height);
        const float alpha = 0.05f;
        float lowpass_height = alpha * median_height + (1.0f - alpha) * last_filtered_height_;
        const float dt = 0.02f;
        const float max_rate = 5.0f * dt;  
        float delta = lowpass_height - last_filtered_height_;
        float rate_limited_height;
        if (std::abs(delta) > max_rate) {
            rate_limited_height = last_filtered_height_ + std::copysign(max_rate, delta);
        } else {
            rate_limited_height = lowpass_height;
        }
        
        barometric_height_ = rate_limited_height;
        last_filtered_height_ = rate_limited_height;
        std::cout << "--------------------------------------------------\n";
        RCLCPP_INFO(this->get_logger(), "barometric_height: %.3f (raw: %.3f, median: %.3f)", 
                    barometric_height_, raw_height, median_height);
    }
    
    void OffboardController::attitude_callback(const VehicleAttitudeMessage::SharedPtr msg)
    {
        this->current_q_ = msg->q;
    }

    void OffboardController::sensor_combined_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
    {
        
        auto delta_t = 0.01;
        float beta = 0.2f;
        float acc_z = msg->accelerometer_m_s2[2] - GRAVITY_Z_;
        if (is_static_bias_calculated(acc_z) == false) return;
        acc_z = acc_z - acc_static_bias_;
        filtered_acc_z_ = (beta * acc_z) + ((1.0f - beta) * filtered_acc_z_);
        if (std::abs(filtered_acc_z_) < 0.001f) return;
        imu_velo_z_ += filtered_acc_z_ * delta_t;
        imu_height_ += (imu_velo_z_ * delta_t) + (0.5f * filtered_acc_z_ * delta_t * delta_t);
        //std::cout << "imu_height_: " << imu_height_ << std::endl;

        //height_estimator_->update_state(barometric_height_, imu_height_, imu_velo_z_);
    }

    bool OffboardController::is_static_bias_calculated(float acc)
    {
        if(acc_bias_counter_< acc_bias_size_)
        {
            acc_sum_ += acc;
            acc_static_bias_ = acc_sum_ / acc_bias_counter_;
            acc_bias_counter_ += 1;
            return false;
        }
        return true;
        
    }

    void OffboardController::estimate_height()
    {

    }

    void OffboardController::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        this->pre_flight_checks_pass_ = msg->pre_flight_checks_pass;

        //if(this->is_baro_ready_ == false)
        //{
        //    this->pre_flight_checks_pass_ = false;
        //}

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

            /*
            if(this->state_ == State::armed && this->is_armed_)
            {
                if (!waypoints.empty()) 
                {
                    auto& target = waypoints.front();

                    if (is_waypoint_reached(target)) 
                    {
                        waypoints.pop();

                        if (!waypoints.empty()) 
                        {
                            auto& next = waypoints.front();
                            RCLCPP_INFO(this->get_logger(), "Sıradaki Hedef: X:%.2f Y:%.2f Z:%.2f", next.x, next.y, next.z);
                        } 
                        else 
                        {
                            RCLCPP_INFO(this->get_logger(), "TÜM GÖREVLER BİTTİ. HOVER MODUNA GEÇİLDİ.");
                        }
                    }

                    if (!waypoints.empty())
                    {
                        auto& current_target = waypoints.front();
                        this->publish_trajectory_setpoint(current_target.x, current_target.y, current_target.z, current_target.yaw);
                    }
                }
                else 
                {
                    this->publish_trajectory_setpoint(current_x_, current_y_, current_z_, current_yaw_);
                }
            }
            */
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Pre flight check not passed");
        }
    }

    void OffboardController::local_position_callback(const VehicleLocalPositionMessage::SharedPtr msg) 
    {   
        /*
        # Position in local NED frame
        float32 x				# North position in NED earth-fixed frame, (metres)
        float32 y				# East position in NED earth-fixed frame, (metres)
        float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)
        */
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
    }

    bool OffboardController::is_waypoint_reached(const Waypoint& w)
    {
        float distance = sqrt( (w.x - current_x_) * (w.x - current_x_) + (w.y - current_y_) * (w.y - current_y_) + (w.z - current_z_) * (w.z - current_z_) );
        return distance < 0.5 ? true : false;
    }

    

    void OffboardController::create_waypoints()
    {
        Waypoint w0(0.0f, 0.0f, -5.0f, -1.56f);
        Waypoint w1(0.0f, 5.0f, -5.0f, -3.14f);
        Waypoint w2(-1.0f, -5.0f, -5.0f, -1.56f);
        Waypoint w3(-2.0f, 2.0f, -5.0f, 0.0f);
        Waypoint w4(-1.0f, 0.0f, -5.0f, 1.56f);

        this->waypoints.push(w0);
        this->waypoints.push(w1);
        this->waypoints.push(w2);
        this->waypoints.push(w3);
        this->waypoints.push(w4);
    }
}



int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Drone::px4_offboard::OffboardController>("offboard_control");
    rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}