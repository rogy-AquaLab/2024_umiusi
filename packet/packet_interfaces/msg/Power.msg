# BLDC, サーボへの出力
# Nucleoが出力する値そのまま

std_msgs/Header header

# BLDC4つ PWMのパルス幅[us]
uint16[4] bldc
# サーボ4つ PWMのパルス幅[us]
# NOTE: 4つの位置関係はBLDCと揃えること
uint16[4] servo
