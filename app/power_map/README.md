# power_map

`power_map_msg/msg/NormalizedPower`のデータを受け取って`packet_interfaces/msg/Power`に変換するNode

## Nodes

- `power_map`: 上述のとおり

## Executables

- `power-map`: `power_map`Nodeを実行する

## Launches

- `power_map_launch.py`:
  `power_map`Nodeに`config/default_param.yml`でparameterを与えつつ、それを実行する。
  その際`app`namespaceが適用され、出力の`/app/power`topicは`/device/order/power`topicにremapされる
  (出力先は`device/nucleo_communicate_py`を参照)。以下のlaunch引数を持つ
    - `param_file`: パラメータの値を指定するYAMLファイルへのパス。
      デフォルトでは`./config/default_param.yml`が使用される。
