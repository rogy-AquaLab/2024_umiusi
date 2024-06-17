#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_depth/depthspacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Depth>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
