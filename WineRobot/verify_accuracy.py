import rpyc
import time
import math
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

# --- 數學核心 ---
ik = LegoRobotIK()

# --- 歸零確認 ---
print("\n========================================")
print("【 L 型歸零模式 】")
print("1. J2 大手臂：絕對垂直")
print("2. J3 小手臂：絕對水平")
print("========================================")
input("精確歸零後，按 Enter 開始大亂鬥...")

# 歸零
m1.position = 0; m2.position = 0; m3.position = 0
for m in [m1, m2, m3]:
    try: m.stop_action = 'hold'
    except: m.stop_command = 'hold'
    m.stop()

# --- 測試函式 ---
def run_test(case_name, x, y, z):
    print(f"\n🔹 {case_name}")
    print(f"   目標: ({x}, {y}, {z})")
    
    try:
        # 1. 計算指令 (這裡會觸發安全檢查)
        t1, t2, t3 = ik.get_motor_instructions(x, y, z)
        
        # 2. 顯示預測角度
        print(f"   ✅ 計算成功! 安全!")
        print(f"   馬達指令: J1={t1:.0f}, J2={t2:.0f}, J3={t3:.0f}")
        
        # 3. 執行移動
        m1.position_sp = int(t1); m1.speed_sp = 150
        m2.position_sp = int(t2); m2.speed_sp = 120
        m3.position_sp = int(t3); m3.speed_sp = 80 
        
        m1.run_to_abs_pos()
        m2.run_to_abs_pos()
        m3.run_to_abs_pos()
        
        # 等待
        while 'running' in m1.state or 'running' in m2.state:
            time.sleep(0.1)
        time.sleep(1) # 等待震動停止
        
        print(f"   >>> 請測量實際座標 (X, Y, Z)")
        input("   (按 Enter 繼續下一題...)")
        
    except ValueError as e:
        print(f"   ⛔ 系統攔截: {e}")
        print("   (這是預期中的保護機制)")
    except Exception as e:
        print(f"   ❌ 未知錯誤: {e}")

# ==========================================
# 測試劇本 (Test Scenarios)
# ==========================================

# 1. 【安全區 - 基礎題】
# 位於手臂的正前方舒適區，應該要很準
run_test("Case 1: 甜蜜點 (準確度測試)", 200, 0, 200)

# 2. 【Y軸測試 - 旋轉題】
# 這是我們很少測的底座旋轉。
# 預期：手臂向左轉 (或右轉)，X 縮短，Y 變大。
# 請拿尺量 X 和 Y 是否符合
run_test("Case 2: 側向移動 (Y軸測試)", 150, 100, 200)

# 3. 【邊界測試 - 高處】
# 這個點很高 (Z=320)，可能需要小手臂伸很直。
# 你的限制是「向前不超過25度」，這題可能會通過，也可能會被擋下。
run_test("Case 3: 高空作業 (邊界測試)", 100, 0, 320)

# 4. 【極限測試 - 遠處】
# X=300 需要手臂伸很長。
# 你的 J2 限制是「向前不超過 22 度」。
# 預期：這題很有可能會被擋下來 (ValueError)，因為要伸這麼遠通常需要 J2 倒超過 22 度。
run_test("Case 4: 極限延伸 (攔截測試)", 300, 0, 150)

# 5. 【極限測試 - 低處內縮】
# 靠近底座且很低。這通常需要 J3 大幅彎曲。
# 你的 J3 限制是 +/- 25 度。
# 預期：這題應該會被擋下來，因為彎曲角度不夠。
run_test("Case 5: 低處內縮 (攔截測試)", 120, 0, 80)

print("\n測試結束，手臂放鬆。")
m1.stop(stop_action='coast')
m2.stop(stop_action='coast')
m3.stop(stop_action='coast')