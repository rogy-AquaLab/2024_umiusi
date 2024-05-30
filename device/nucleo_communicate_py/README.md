# nucleo communication

Nucleoとの通信周り

TODO: いつかC++に書き換える

## Nodes

- `Receiver`: Nucleoからデータを受け取る
- `Sender`: Nucleoにデータを送る

## Executables

- `receiver`: `Receiver`Nodeをspin
- `sender`: `Sender`Nodeをspin
- `main`: `Receiver`, `Sender`Node2つを同時にspin

## Launches

- `main_launch.py`: `main`Executableを`device`namespace下で実行
- `receiver_launch.py`: `receiver`Executableを`device`namespace下で実行
- `sender_launch.py`: `sender`Executableを`device`namespace下で実行
