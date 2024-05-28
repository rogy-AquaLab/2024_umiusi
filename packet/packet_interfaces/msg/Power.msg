# BLDC, サーボへの出力
# Nucleoが出力する値そのまま

# BLDC4つ PWMのパルス幅[ms]
uint16[4] bldc
# サーボ4つ PWMのパルス幅[ms]
# NOTE: 4つの位置関係はBLDCと揃えること
uint16[4] servo
