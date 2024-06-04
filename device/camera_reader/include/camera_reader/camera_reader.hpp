#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraReader : public rclcpp::Node {
public:
    CameraReader();

private:
    void timer_callback();

    image_transport::Publisher   publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture             cap_{ 0 }; // Open default camera
};
