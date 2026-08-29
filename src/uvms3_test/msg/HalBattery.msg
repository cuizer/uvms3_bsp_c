# 时间戳
int64 timestamp
# 电池状态
byte battery_status_48v
byte battery_status_72v
# 电池电压 (单位: 0.1V)
uint16 battery_voltage_48v
uint16 battery_voltage_72v
# 电池电流 (单位: 0.1A)
int16 battery_current_48v
int16 battery_current_72v
# 循环次数
uint16 cycle_count_48v
uint16 cycle_count_72v
# 电池温度 (单位: 1℃)
uint16 battery_temperature_48v
uint16 battery_temperature_72v
# 剩余电量 (单位: 0.1AH)
uint16 remain_capacity_48v
uint16 remain_capacity_72v
# 总电量 (单位: 0.1AH)
uint16 total_capacity_48v
uint16 total_capacity_72v
# 开关状态: 0=关闭, 1=打开
byte switch_state_12v
byte switch_state_24v
byte switch_state_72v
