#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_current/currentspacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Current>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
