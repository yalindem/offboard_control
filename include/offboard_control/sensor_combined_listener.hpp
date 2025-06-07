#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

namespace px4_offboard
{
    class SensorCombinedListener : public rclcpp::Node
    {
        public:

            explicit SensorCombinedListener();

        private:

        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_;

    };
}