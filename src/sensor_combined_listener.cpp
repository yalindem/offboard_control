#include "offboard_control/sensor_combined_listener.hpp"


namespace px4_offboard
{
    SensorCombinedListener::SensorCombinedListener():rclcpp::Node("sensor_combined_listener")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
        [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
        std::cout << "============================="   << std::endl;
        std::cout << "ts: "          << msg->timestamp    << std::endl;
        std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
        std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
        std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
        std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
        std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
        std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
        std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
        std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
        std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
        }); 

        gps_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", qos,
        [this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        std::cout << "RECEIVED SENSOR GPS DATA"   << std::endl;
        std::cout << "============================="   << std::endl;
        std::cout << "ts: "          << msg->timestamp    << std::endl;
        std::cout << "latitude_deg: " << msg->latitude_deg  << std::endl;
        std::cout << "longitude_deg: " << msg->longitude_deg  << std::endl;
        std::cout << "satellites_used: " << msg->satellites_used  << std::endl;
        std::cout << "heading: " << msg->heading  << std::endl;
        
        }); 
    }
}
int main(int argc, char* argv[])
{
    std::cout << "Starting sensor_combined listener node ...\n";
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_offboard::SensorCombinedListener>());
    return 0;
}