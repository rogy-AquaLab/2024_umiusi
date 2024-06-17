#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_power/powerspacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Power>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
