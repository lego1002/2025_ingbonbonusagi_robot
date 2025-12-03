import tkinter as tk
from tkinter import ttk
import rpyc
import time
from kinematics import LegoRobotIK

# ==========================================
# 設定區
# ==========================================
EV3_IP = '172.20.10.2'
INVERT_Y_AXIS = False  # 如果覺得 Y 軸方向反了，改成 True

# ==========================================
# 1. 連線與初始化
# ==========================================
print(f"正在連線 UI 到 EV3 ({EV3_IP})...")
try:
    conn = rpyc.classic.connect(EV3_IP)
    print("✅ 連線成功！")
except Exception as e:
    print(f"❌ 連線失敗: {e}")
    exit()

try:
    ev3_motor = conn.modules['ev3dev2.motor']
except:
    ev3_motor = conn.modules['ev3dev.ev3']

m1 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_A)
m2 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_B)
m3 = ev3_motor.LargeMotor(ev3_motor.OUTPUT_C)

ik = LegoRobotIK()

# ==========================================
# 2. 視窗介面
# ==========================================
root = tk.Tk()
root.title("Universal Robot Controller")
root.geometry("500x750")

# 全域變數 (滑桿用)
target_x = tk.DoubleVar(value=0) 
target_y = tk.DoubleVar(value=0)
target_z = tk.DoubleVar(value=0)

# 校正用變數 (預設填入你習慣的停車點)
calib_x = tk.DoubleVar(value=168.0)
calib_y = tk.DoubleVar(value=0.0)
calib_z = tk.DoubleVar(value=260.0)

status_label = tk.Label(root, text="⚠️ 請輸入當前座標並按下 [校正]", fg="red", font=("Arial", 12, "bold"))
status_label.pack(pady=10)

# ==========================================
# 3. 核心邏輯
# ==========================================
def move_arm():
    """發送指令 (只有拉動滑桿時觸發)"""
    x = target_x.get()
    raw_y = target_y.get()
    y = -raw_y if INVERT_Y_AXIS else raw_y
    z = target_z.get()
    
    try:
        t1, t2, t3 = ik.get_motor_instructions(x, y, z)
        
        info_text.set(f"目標: ({x:.0f}, {y:.0f}, {z:.0f})\n"
                      f"指令: {t1:.0f}, {t2:.0f}, {t3:.0f}")
        status_label.config(text="移動中...", fg="blue")
        
        m1.position_sp = int(t1); m1.speed_sp = 200
        m2.position_sp = int(t2); m2.speed_sp = 200
        m3.position_sp = int(t3); m3.speed_sp = 150
        
        m1.run_to_abs_pos()
        m2.run_to_abs_pos()
        m3.run_to_abs_pos()
        
    except ValueError as e:
        status_label.config(text=f"⛔ {e}", fg="red")
    except Exception as e:
        status_label.config(text=f"❌ 錯誤: {e}", fg="red")

def on_slider_release(event):
    move_arm()

def nudge(axis, amount):
    if axis == 'x': target_x.set(target_x.get() + amount)
    elif axis == 'y': target_y.set(target_y.get() + amount)
    elif axis == 'z': target_z.set(target_z.get() + amount)
    move_arm()

# --- 請替換 gui_universal.py 中的這個函式 ---

def calibrate_custom():
    """
    【萬能校正 - 靜音版】
    設定當前位置為輸入的座標，且馬達完全不會出力震動。
    """
    cx = calib_x.get()
    cy = calib_y.get()
    cz = calib_z.get()
    
    print(f"正在以自訂座標 ({cx}, {cy}, {cz}) 進行靜音校正...")
    
    try:
        # 1. 用 FK 反推理想角度 (程式碼不變...)
        import math
        # J1
        theta1 = math.degrees(math.atan2(cy, cx))
        # 投影
        r_ground = (cx**2 + cy**2)**0.5 - ik.a1
        z_arm = cz - ik.d1
        reach = (r_ground**2 + z_arm**2)**0.5
        # 三角函數
        alpha = math.degrees(math.atan2(z_arm, r_ground))
        cos_beta = (ik.a2**2 + reach**2 - ik.a3**2) / (2 * ik.a2 * reach)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.degrees(math.acos(cos_beta))
        theta2_phys = alpha + beta
        cos_gamma = (ik.a2**2 + ik.a3**2 - reach**2) / (2 * ik.a2 * ik.a3)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.degrees(math.acos(cos_gamma))
        theta3_phys = -(180 - gamma)
        
        # 算出馬達應該在的編碼值
        target_m1 = theta1 * ik.DIR_J1 * ik.GR_J1
        target_m2 = (theta2_phys - ik.OFFSET_J2) * ik.DIR_J2 * ik.GR_J2
        target_m3 = (theta3_phys - ik.OFFSET_J3) * ik.DIR_J3 * ik.GR_J3

        # 2. 【關鍵修改】設定馬達，但使用 'coast' 模式
        # 先放鬆馬達，避免設定位置時產生對抗力矩
        for m in [m1, m2, m3]:
            try: m.stop_action = 'coast' # 改成滑行模式
            except: m.stop_command = 'coast'
            m.stop()
            
        # 設定編碼器數值
        time.sleep(0.1) # 稍微等一下讓馬達完全放鬆
        m1.position = int(target_m1)
        m2.position = int(target_m2)
        m3.position = int(target_m3)
        
        # 再次確認放鬆
        for m in [m1, m2, m3]:
            m.stop()

        # 3. 同步滑桿
        target_x.set(cx)
        target_y.set(cy)
        target_z.set(cz)
        
        status_label.config(text=f"✅ 靜音校正成功! 當前: ({cx:.0f}, {cy:.0f}, {cz:.0f})", fg="green")
        info_text.set("座標系統已同步 (馬達放鬆狀態)")
        
    except Exception as e:
        status_label.config(text=f"校正失敗: {e}", fg="red")
def stop_all():
    m1.stop(); m2.stop(); m3.stop()
    status_label.config(text="⛔ 已緊急停止", fg="red")

# ==========================================
# 4. 介面佈局
# ==========================================

# --- 校正區塊 (最上方) ---
calib_frame = tk.LabelFrame(root, text="1. 初始位置校正 (輸入目前實際座標)", fg="blue")
calib_frame.pack(padx=20, pady=10, fill="x")

c_row = tk.Frame(calib_frame)
c_row.pack(pady=5)
tk.Label(c_row, text="X:").pack(side="left")
tk.Entry(c_row, textvariable=calib_x, width=6).pack(side="left", padx=5)
tk.Label(c_row, text="Y:").pack(side="left")
tk.Entry(c_row, textvariable=calib_y, width=6).pack(side="left", padx=5)
tk.Label(c_row, text="Z:").pack(side="left")
tk.Entry(c_row, textvariable=calib_z, width=6).pack(side="left", padx=5)

tk.Button(calib_frame, text="設定以此座標為起點 (Set Home)", command=calibrate_custom, bg="lightblue").pack(pady=5, fill="x", padx=10)


# --- 控制區塊 ---
frame = ttk.LabelFrame(root, text="2. 座標控制 (mm)")
frame.pack(padx=20, pady=10, fill="x")

def create_control_row(label_text, var, min_v, max_v, axis_char):
    row = tk.Frame(frame)
    row.pack(fill='x', pady=5)
    tk.Label(row, text=label_text, width=15, anchor='w').pack(side='left')
    tk.Button(row, text="<", command=lambda: nudge(axis_char, -5)).pack(side='left')
    scale = ttk.Scale(row, from_=min_v, to=max_v, variable=var, orient='horizontal')
    scale.pack(side='left', fill='x', expand=True, padx=5)
    scale.bind("<ButtonRelease-1>", on_slider_release)
    tk.Button(row, text=">", command=lambda: nudge(axis_char, 5)).pack(side='left')

create_control_row("X (前後)", target_x, 50, 350, 'x')
create_control_row("Y (左右)", target_y, -250, 250, 'y') 
create_control_row("Z (上下)", target_z, 0, 380, 'z')

info_text = tk.StringVar(value="請先進行校正")
tk.Label(frame, textvariable=info_text, bg="#eee", relief="sunken", height=3).pack(fill='x', pady=10)

tk.Button(root, text="緊急停止 (STOP)", command=stop_all, bg="red", fg="white", font=("Arial", 12, "bold"), height=2).pack(pady=10, padx=20, fill="x")

root.mainloop()