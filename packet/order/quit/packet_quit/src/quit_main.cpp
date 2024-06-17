#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_quit/quitspacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Quit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
