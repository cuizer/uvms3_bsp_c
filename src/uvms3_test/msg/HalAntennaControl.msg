uint8 cmd_type            # 指令码 (0:上升, 1:下降, 2:仅解锁, 3:自定义系数, 4:急停, 5:解除急停)
float32 target_coeff      # 目标升降系数 (0.0 ~ 255.0，仅当 cmd_type==3 时生效)
