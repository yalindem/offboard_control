#include "offboard_control/edge_detector_node.hpp"

namespace px4_offboard
{
    EdgeDetectorNode::EdgeDetectorNode(std::string node_name) : Node(node_name)
    {
        using std::placeholders::_1;
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/world/walls/model/x500_mono_cam_0/link/camera_link/sensor/imager/image", 10, std::bind(&EdgeDetectorNode::image_callback, this, _1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/edge_image", 10);

        RCLCPP_INFO(this->get_logger(), "Edge Detector Node Started");
    }

    void EdgeDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } 
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Edge Detection
        cv::Mat gray, edges;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);
        cv::Canny(gray, edges, 50, 150);

        // Publish the result
        cv_bridge::CvImage edge_msg;
        edge_msg.header = msg->header;
        edge_msg.encoding = "mono8";
        edge_msg.image = edges;

        image_pub_->publish(*edge_msg.toImageMsg());
    }

}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_offboard::EdgeDetectorNode>("edge_detector_node"));
  rclcpp::shutdown();
  return 0;
}
