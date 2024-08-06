# pi_led

ラズパイのGPIOでLED(フルカラー)を扱う

## Nodes

- `Led`: LEDを扱う。以下のparameterを(起動時のみ)受け取る
    - `led_pin_r`: Redのピン
    - `led_pin_g`: Greenのピン
    - `led_pin_b`: Blueのピン

parameterの設定例は./config以下を参照

## Executables

- `pi_led`: `Led`Nodeをspinする

## Launches

- `led_launch.py`: `Led`Nodeを`device`namespace下でspinする。以下のlaunch引数が与えられる
    - `variant`: 使用するGPIO Pinセットの略称。`default`, `right`, `left`, `custom`のいずれか
        - `default`: ./config/default_param.ymlのparameter設定でspinする。
          topic`/device/led_color`が`/packet/order/led_color`にremapされる
        - `right`: ./config/default_param.ymlのparameter設定でspinする。
          Node名が`led_right`に書きかわり、
          topic`/device/led_color`が`/packet/order/led_color_right`にremapされる
        - `left`: ./config/left_param.ymlのparameter設定でspinする。
          Node名が`led_left`に書きかわり、
          topic`/device/led_color`が`/packet/order/led_color_left`にremapされる
        - `custom`: launch引数`param_file`で指定されたparameterでspinする。
          Node名が`led_custom`に書きかわり、
          topic`/device/led_color`が`/packet/order/led_color_custom`にremapされる
    - `param_file`: カスタムのparameter設定ファイル。`variant:=custom`の時のみ有効
