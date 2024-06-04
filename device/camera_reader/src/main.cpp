#include "my_camera_cpp_package/camera_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
