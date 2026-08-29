int64 timestamp           # 系统时间戳 (纳秒)
uint8 brake_status        # 刹车状态 (0: 锁定/断电, 1: 释放/通电)
uint8 running_status      # 运行状态 (0: 停止, 1: 运动中)
float64 total_angle       # 实时位置系数 (逆向换算出的 0.0 ~ 255.0)
