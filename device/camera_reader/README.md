# camera_reader

USBカメラの映像を取得

## Nodes

- `Camera`: カメラ映像を取得

## Executables

- `camera`: `Camera`Nodeをspin

## Launches

- `camera_reader_launch.py`: `Camera`Nodeをspinする。以下のlaunch引数が与えられる
    - `index`: カメラの番号。
      与えられた場合は`/dev/video{index}`に該当するカメラを参照し、
      topic`/app/camera_image`が`/packet/camera_image_{index}`にremapされる。
      与えられなかった場合は`/dev/video0`に該当するカメラを参照し、
      topic`/app/camera_image`が`/packet/camera_image`にremapされる。
