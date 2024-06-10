# power_map_msg

正規化されたBLDC, サーボの値

## msg

- `NormalizedPower`
    - `float32[4] bldc`: 各値 $v$ に対して $-1 \le v \le 1$
    - `float32[4] servo`: 各値 $v$ に対して $-\pi \le v \le \pi$
