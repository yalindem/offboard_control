#ifndef CPP_OFFBOARD_CONTROLLER
#define CPP_OFFBOARD_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <queue>

#include "px4_msgs/srv/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/sensor_baro.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <px4_msgs/msg/sensor_gps.hpp>


#include "height_estimator.hpp"

using VehicleCommandSrv = px4_msgs::srv::VehicleCommand;
using VehicleCommandClient = rclcpp::Client<VehicleCommandSrv>;
using VehicleCommandSharedPtr = VehicleCommandClient::SharedPtr;
using VehicleCommandSharedFuture = VehicleCommandClient::SharedFuture;
using VehicleCommandMessage = px4_msgs::msg::VehicleCommand;
using VehicleCommandMessageAck = px4_msgs::msg::VehicleCommandAck;
using VehicleStatusMessage = px4_msgs::msg::VehicleStatus;
using VehicleAttitudeMessage = px4_msgs::msg::VehicleAttitude;
using VehicleCommandMessageSubscriber = rclcpp::Subscription<VehicleStatusMessage>::SharedPtr;
using VehicleAttitudeMessageSubscriber = rclcpp::Subscription<VehicleAttitudeMessage>::SharedPtr;
using TrajectorySetpointMessage = px4_msgs::msg::TrajectorySetpoint;
using TrajectorySetpointPublisher = rclcpp::Publisher<TrajectorySetpointMessage>::SharedPtr;
using OffboardControlModeMessage = px4_msgs::msg::OffboardControlMode;
using OffboardControlModePublisher = rclcpp::Publisher<OffboardControlModeMessage>::SharedPtr;
using VehicleSensorBarometerMessage = px4_msgs::msg::SensorBaro;
using VehicleSensorBarometerSubscriber = rclcpp::Subscription<VehicleSensorBarometerMessage>::SharedPtr;
using SensorCombinedMessage = px4_msgs::msg::SensorCombined;
using SensorCombinedSubscriber = rclcpp::Subscription<SensorCombinedMessage>::SharedPtr;
using VehicleSensorGPSMessage = px4_msgs::msg::SensorGps;
using VehicleSensorGPSSubscriber = rclcpp::Subscription<VehicleSensorGPSMessage>::SharedPtr;
using VehicleLocalPositionMessage = px4_msgs::msg::VehicleLocalPosition;
using VehicleLocalPositionSubscriber = rclcpp::Subscription<VehicleLocalPositionMessage>::SharedPtr;

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

struct Waypoint
{
    float x;
    float y;
    float z;
    float yaw;
    Waypoint(float x_, float y_, float z_, float yaw_) 
        : x(x_), y(y_), z(z_), yaw(yaw_) {}
};

#define GREEN(text) "\033[1;32m" text "\033[0m"

namespace Drone::px4_offboard
{

    class OffboardController : public rclcpp::Node
    {
        public:
            OffboardController(std::string node_name);
            void run();

        private:
            VehicleCommandSharedPtr vehicle_command_client_;
            std::unique_ptr<Estimator::HeightEstimator> height_estimator_;
            
            void arm();
            void request_vehicle_command(std::uint16_t command, float param1 = 0.0, float param2 = 0.0);
            float calculate_barometric_height(const float pressure, const float temp);
            void switch_to_offboard_mode();
            void publish_trajectory_setpoint(float x, float y, float z, float yaw);
            void publish_offboard_control_mode();
            void response_callback(VehicleCommandSharedFuture future);
            void vehicle_sensor_barometer_callback(const VehicleSensorBarometerMessage::SharedPtr msg);
            void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
            void sensor_combined_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
            void vehicle_sensor_gps_callback(const VehicleSensorGPSMessage::SharedPtr msg);
            void attitude_callback(const VehicleAttitudeMessage::SharedPtr msg);
            void local_position_callback(const VehicleLocalPositionMessage::SharedPtr msg);
            void estimate_height();
            void create_waypoints();
            bool is_waypoint_reached(const Waypoint& w);

            bool is_static_bias_calculated(float acc);
            //float get_distance();

            VehicleCommandMessageSubscriber vehicle_command_sub_;
            VehicleSensorBarometerSubscriber vehicle_sensor_baro_sub_;
            SensorCombinedSubscriber vehicle_sensor_imu_sub_;
            VehicleSensorGPSSubscriber vehicle_sensor_gps_sub_;
            VehicleAttitudeMessageSubscriber vehicle_attitude_sub_;
            VehicleLocalPositionSubscriber local_pos_sub_;
            

            bool arming_requested_{false};

            State state_ {State::pre_flight};
            NavState nav_state_ {NavState::manual};

            TrajectorySetpointPublisher trajectory_pub_;
            OffboardControlModePublisher offboard_mode_pub_;

            rclcpp::TimerBase::SharedPtr timer_;

            bool pre_flight_checks_pass_{false};
            bool is_armed_{false};
            bool is_baro_ready_{false};
            bool is_imu_ready {false};

            float barometric_height_ {0.0f};
            float imu_velo_z_;
            float imu_height_{0.0f};
            float initial_pressure_ {-1.0f};
            float initial_temp_ {0.0f};
            float filtered_acc_z_ {0.0f};
            float height_est_{0.0f};
            float acc_static_bias_{0.0f};
            float acc_sum_{0.0f};
            size_t acc_bias_size_{200};
            size_t acc_bias_counter_{200};
            std::queue<Waypoint> waypoints;

            std::array<float, 4> current_q_;

            rclcpp::Time prev_imu_time_;

            float current_x_{0.0f}, current_y_{0.0f}, current_z_{0.0f}, current_yaw_{0.0};
            
    };

}

#endif