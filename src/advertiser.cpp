#include "offboard_control/advertiser.hpp"

using namespace std::chrono_literals;

namespace px4_offboard
{
    Advertiser::Advertiser() : Node("debug_vect_advertiser")
    {
        publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("fmu/debug_vect/in", 10);
        auto timer_callback =
        [this]()->void {
        auto debug_vect = px4_msgs::msg::DebugVect();
        debug_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
        std::string name = "test";
        std::copy(name.begin(), name.end(), debug_vect.name.begin());
        debug_vect.x = 1.0;
        debug_vect.y = 2.0;
        debug_vect.z = 3.0;
        RCLCPP_INFO(this->get_logger(), "\033[97m Publishing debug_vect: time: %llu x: %f y: %f z: %f \033[0m",
                                        debug_vect.timestamp, debug_vect.x, debug_vect.y, debug_vect.z);
        this->publisher_->publish(debug_vect);
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

}

int main(int argc, char *argv[])
{
    std::cout << "Starting debug_vect advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_offboard::Advertiser>());

    rclcpp::shutdown();
    return 0;
}