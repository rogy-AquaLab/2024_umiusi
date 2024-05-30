# nucleo communication

Raspberry PiのI2C接続

## Nodes

- `Depth`: 深さセンサー
- `Imu`: IMU

## Executables

- `depth`: `Depth`Nodeをspin
- `imu`: `Imu`Nodeをspin
- `all`: `Depth`, `Imu`Node2つを同時にspin

## Launches

- `all_launch.py`: `all`executableを`device`namespace下で実行
- `depth_launch.py`: `depth`executableを`device`namespace下で実行
- `imu_launch.py`: `imu`executableを`device`namespace下で実行
