#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>


namespace px4_offboard
{
    class SensorCombinedListener : public rclcpp::Node
    {
        public:
            explicit SensorCombinedListener();

        private:
            rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_;
            rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
    };
}