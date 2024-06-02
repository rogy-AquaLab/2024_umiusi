// listener_main.cppと同じなのでコメントは略

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "packet_composed/sensorspacket.hpp"  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // ROS2プロセスの初期化
    auto node = std::make_shared<composed>();
    // `listener`nodeを作成
    rclcpp::spin(node);
    // 作成したnodeを実行
    rclcpp::shutdown();
   // ROS2プロセスを終了
    return 0;
}