# Joystick

ゲームコントローラー

## Nodes

- `Joystick`: ゲームコントローラーを読み取って`joystick`topicに`sensor_msgs/msg/Joy`型でpublishする

## Executables

- `joystick`: `Joystick`Nodeを起動する

## Launches

- `joystick_launch.py`: `Joystick`Nodeを`device`namespace下で起動する
