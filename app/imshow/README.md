# imshow

USBカメラの映像を映す

## Nodes

- `Imshow`: 映像を映す

## Executables

- `imshow`: `Imshow`Nodeをspin

## Launches

- `imshow_launch.py`: `Imshow`Nodeをspinする。以下のlaunch引数をとる
    - `index`: 受け取る画像のインデックス。
      与えられた場合はtopic`/app/camera_image`が`/packet/camera_image_{index}`にremapされ、
      与えられなかった場合はtopic`/app/camera_image`が`/packet/camera_image`にremapされる。
