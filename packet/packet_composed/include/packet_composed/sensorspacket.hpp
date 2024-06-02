#include <memory>

#include "rclcpp/rclcpp.hpp"
// talker.cppでmsgを使えるようなincludeの構文
#include "std_msgs/msg/string.hpp"

// `composed`nodeを扱うクラス
class composed : public rclcpp::Node {
private:
    // `chatter`topicのpublisher
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    // 定期的に実行するためのタイマー
    std::shared_ptr<rclcpp::TimerBase> _timer;
    // messageをpublishするごとにカウントアップさせる
    unsigned int _count;

    // タイマーで定期的に実行する関数 宣言
    void _loop();

public:
    // コンストラクタ
    composed();
};