#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageReceiverNode : public rclcpp::Node {
public:
    ImageReceiverNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};
