import rpyc
import time
from kinematics import LegoRobotIK

# --- 連線設定 ---
EV3_IP = '172.20.10.2'
print(f"連線至 {EV3_IP}...")
conn = rpyc.classic.connect(EV3_IP)

try:
    ev3_motor = conn.modules['ev3dev2.motor']
except:
    ev3_motor = conn.modules['ev3dev.ev3']

m1 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_A)
m2 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_B)
m3 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_C)

ik = LegoRobotIK()

print("\n========================================")
print("【 邊界極限測試 】")
print("請將手臂 L 型歸零 (J2垂直, J3水平)")
print("========================================")
input("按 Enter 開始...")

# 歸零
m1.position = 0; m2.position = 0; m3.position = 0
for m in [m1, m2, m3]:
    try: m.stop_action = 'hold'
    except: m.stop_command = 'hold'
    m.stop()

def try_move(name, x, y, z):
    print(f"\n--- 測試: {name} ({x}, {y}, {z}) ---")
    try:
        t1, t2, t3 = ik.get_motor_instructions(x, y, z)
        print(f"✅ 計算成功 (在安全範圍內)")
        print(f"   J2={t2:.0f}, J3={t3:.0f}")
        
        # 執行移動
        m1.position_sp = int(t1); m1.speed_sp = 150
        m2.position_sp = int(t2); m2.speed_sp = 150
        m3.position_sp = int(t3); m3.speed_sp = 100
        
        m1.run_to_abs_pos()
        m2.run_to_abs_pos()
        m3.run_to_abs_pos()
        
        # 等待
        while 'running' in m1.state or 'running' in m2.state:
            time.sleep(0.1)
        print("   -> 移動完成")
        
    except ValueError as e:
        print(f"⛔ 被安全系統攔截: {e}")
        print("   -> 這是好事！防止了馬達撞牆")
    except Exception as e:
        print(f"❌ 未知錯誤: {e}")

# --- 開始測試 ---

# 1. 安全區測試 (應該要能動)
# L型歸零點附近，稍微往前
try_move("安全點 - 稍微前伸", 200, 0, 200)
time.sleep(1)

# 2. J2 前傾極限測試
# 你設定 J2 最多往前 22 度 (到 68 度)
# 我們試著去一個很遠很低的地方，這通常需要 J2 大幅前傾
try_move("J2 極限測試 - 超遠伸展", 300, 0, 100) 
# 預期：如果這個點需要 J2 < 68 度，程式應該會報錯擋下來

# 3. J3 彎曲極限測試
# 你設定 J3 只能動 +/- 25 度
# 我們試著去一個很高很近的地方，這通常需要 J3 大幅彎曲
try_move("J3 極限測試 - 高處收縮", 100, 0, 350)
# 預期：如果需要 J3 抬太高，程式會報錯

print("\n測試結束。")
input("按 Enter 放鬆馬達...")
m1.stop(stop_action='coast')
m2.stop(stop_action='coast')
m3.stop(stop_action='coast')