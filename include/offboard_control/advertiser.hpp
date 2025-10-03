#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <px4_msgs/msg/debug_vect.hpp>

using namespace std::chrono_literals;

namespace px4_offboard
{
    class Advertiser : public rclcpp::Node
    {
        public:
            Advertiser();
        
        private: 
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
    };
}