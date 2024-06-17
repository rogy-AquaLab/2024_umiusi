#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_voltage/voltagepacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Voltage>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
