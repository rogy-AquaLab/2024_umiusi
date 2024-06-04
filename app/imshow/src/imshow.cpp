#include "imshow/imshow.hpp"

Imshow::Imshow() : Node("imshow") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image",
        10,
        std::bind(&Imshow::image_callback, this, std::placeholders::_1)
    );
}

void Imshow::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::imshow("Received Image", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}
