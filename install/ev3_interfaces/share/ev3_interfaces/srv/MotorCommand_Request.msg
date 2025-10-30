# MotorCommand.srv
# 通用馬達控制命令
# 可指定模式（RUN, STOP, RESET, DUTY, REL, ABS）

string mode           # 指令名稱
int32 motor_id        # 馬達ID (0~5)，或 -1 代表全體
float32 value1        # 參數1 (duty/degree)
float32 value2        # 參數2 (speed)
