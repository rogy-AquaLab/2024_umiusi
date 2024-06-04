# nucleo communication

Nucleoとの通信周り

TODO: いつかC++に書き換える

## Nodes

- `Receiver`: Nucleoからデータを受け取る
- `Sender`: Nucleoにデータを送る
- `Channel`: データの送受信両方を行う

## Executables

- `receiver`: `Receiver`Nodeをspin
- `sender`: `Sender`Nodeをspin
- `channel`: `Channel`Nodeをspin

## Launches

- `channel_launch.py`: `channel`Executableを`device`namespace下で実行
- `receiver_launch.py`: `receiver`Executableを`device`namespace下で実行
- `sender_launch.py`: `sender`Executableを`device`namespace下で実行
