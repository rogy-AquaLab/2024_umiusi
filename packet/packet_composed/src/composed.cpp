#include <chrono>  // 時間ユーティリティ タイマー作成時に使う
#include <functional>  // 関数型プログラミング笑
#include <sstream>  // stringstream iostreamみたいなやつ
#include <string>  // みんな大好き文字列

#include "packet_composed/sensorspacket.hpp"  // composedクラス

// タイマーで定期的に実行する関数 宣言
void composed::_loop() {
    // 文字列ストリーム
    // std::coutの書き込み先がstd::stringになったみたいな感じ
    std::stringstream ss;
    // 構築 "Hello, world! 0" みたいになる
    ss << "Hello, world! " << _count;
    // 将来publishするmessage#include "packet_interfaces/msg/Composed.msg" 
    std_msgs::msg::String msg{};
    // std::string型のデータを代入
    msg.data = ss.str();
    // ログ出力
    RCLCPP_INFO(this->get_logger(), "say %s", ss.str().c_str());
    // RCLCPP_INFO_STREAM(this->get_logger(), "say: [" << msg.data << "]");
    // messageをpublish
    _publisher->publish(msg);
    // _countをインクリメント publishするたびにカウントが増加するようになる
    _count++;
}

// composedコンストラクタ 定義
composed::composed() :
    rclcpp::Node("composed"),  // node名をcomposedに設定
    _publisher(),  // _publisherのshared_ptrをnullptrで初期化
    _timer(),  // タイマーをnullptrで初期化
    _count(0)  // カウントの初期値は0
{
    // これで50msとかかけるようになる
    using namespace std::chrono_literals;
    // publisherを作成
    // `chatter`topicにQoS 10で出力すると設定
    _publisher = this->create_publisher<std_msgs::msg::String>("/sensorspacket", 10);
    // タイマーで定期的に呼び出される関数
    auto loop = std::bind(&composed::_loop, this);
    // タイマー作成
    // 500ミリ秒ごとにloopが呼び出される
    _timer = this->create_wall_timer(500ms, loop);
}