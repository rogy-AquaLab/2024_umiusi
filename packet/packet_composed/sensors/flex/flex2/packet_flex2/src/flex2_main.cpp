#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_flex2/flex1spacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Flex2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
