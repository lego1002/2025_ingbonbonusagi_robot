import rpyc
import time
from kinematics import LegoRobotIK

# --- 連線設定 ---
EV3_IP = '172.20.10.2' 
conn = rpyc.classic.connect(EV3_IP)
print(f"已連線 EV3: {EV3_IP}")

try:
    ev3_motor = conn.modules['ev3dev2.motor']
except:
    ev3_motor = conn.modules['ev3dev.ev3']

m1 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_A)
m2 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_B)
m3 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_C)

# --- ⚠️ L型歸零儀式 ⚠️ ---
print("========================================")
print("【 L 型歸零模式 】")
print("1. 大手臂 (J2)：垂直朝天 (90度)")
print("2. 小手臂 (J3)：水平向前 (-90度)")
print("========================================")
input("擺好後按 Enter 歸零...")

m1.position = 0
m2.position = 0
m3.position = 0

for m in [m1, m2, m3]:
    try: m.stop_action = 'hold'
    except: m.stop_command = 'hold'
    m.stop() 

print("歸零完成。")
ik = LegoRobotIK()

# --- 移動函式 (柔順版) ---
def go(x, y, z, speed):
    print(f"\n--- 前往 ({x}, {y}, {z}) ---")
    try:
        t1, t2, t3 = ik.get_motor_instructions(x, y, z)
        print(f"目標角度: {t1:.0f}, {t2:.0f}, {t3:.0f}")
        
        # J1 底座
        m1.position_sp = int(t1)
        m1.speed_sp = int(speed * 10) # 底座穩，速度中等
        
        # J2 大手臂 (負載最大)
        m2.position_sp = int(t2)
        m2.speed_sp = int(speed * 8)  # 稍微慢一點增加扭力
        
        # J3 小手臂 (最不穩)
        m3.position_sp = int(t3)
        m3.speed_sp = int(speed * 5)  # [關鍵修正] 大幅降速，避免甩動
        
        # 發送指令
        m1.run_to_abs_pos()
        m2.run_to_abs_pos()
        m3.run_to_abs_pos()
        
        # 等待
        while 'running' in m1.state or 'running' in m2.state or 'running' in m3.state:
            time.sleep(0.1)
            
        print("到達目標點！")
        
    except Exception as e:
        print(f"錯誤: {e}")

# --- 測試區 ---
if __name__ == "__main__":
    try:
        # 再次測試最大高度 (這次動作會很溫柔)
        # 如果上次是 354mm，我們這次設個保守目標 340mm 看看穩不穩
        go(50, 0, 340, speed=15)
        
    except KeyboardInterrupt:
        m1.stop(); m2.stop(); m3.stop()