#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_composed/sensorspacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<composed>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
