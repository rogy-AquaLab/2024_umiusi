# packet_interfaces

device-app間で受け渡すメッセージ型

## Messages

- Composed: センサー類の値まとめ
- Current: 電流センサーの値
- Depth: 深さセンサーの値
- Flex: まげセンサーの値
- LedColor: LEDの出力色
- Power: BLDCとサーボへの出力値 (Composedに含まれない)
- Voltage: 電圧計の値

(see also)

- sensor_msgs/msg/Image: カメライメージ (**これはComposedに含めない**)
- std_msgs/msg/Empty: quit命令の型
