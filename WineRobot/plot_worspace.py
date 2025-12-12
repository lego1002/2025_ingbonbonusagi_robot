import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==========================================
# 1. 機器人參數定義 (必須與 kinematics.py 一致)
# ==========================================
# 機構長度 (mm)
d1 = 55.79
a1 = 23.91
a2 = 134.0
a3 = 186.0

# 軟體角度限制 (Software Limits)
# J1 (底座): 假設可以左右轉 170 度
LIMIT_J1_MIN, LIMIT_J1_MAX = -170, 170

# J2 (大臂): [68, 123]
LIMIT_J2_MIN = 90.0 - 22.0
LIMIT_J2_MAX = 90.0 + 33.0

# J3 (小臂): [-115, -65]
LIMIT_J3_MIN = -90.0 - 25.0
LIMIT_J3_MAX = -90.0 + 25.0

# ==========================================
# 2. 正向運動學核心 (NumPy 高速版)
# ==========================================
def forward_kinematics_np(j1_deg, j2_deg, j3_deg):
    """
    輸入大量角度陣列，快速計算對應的 (x,y,z) 座標陣列
    """
    # 將角度轉為弧度
    t1 = np.radians(j1_deg)
    t2 = np.radians(j2_deg)
    # 注意: J3 是相對角度，幾何上的角度是 t2 + t3
    t_sum = np.radians(j2_deg + j3_deg)
    
    # 計算水平投影半徑 R
    r_ground = a1 + a2 * np.cos(t2) + a3 * np.cos(t_sum)
    
    # 計算座標
    x = r_ground * np.cos(t1)
    y = r_ground * np.sin(t1)
    z = d1 + a2 * np.sin(t2) + a3 * np.sin(t_sum)
    
    return x, y, z

# ==========================================
# 3. 產生數據點雲
# ==========================================
print("正在生成工作空間點雲，請稍候...")
NUM_POINTS = 30000  # 產生 3 萬個點，越多越密

# 在限制範圍內隨機採樣角度
j1_samples = np.random.uniform(LIMIT_J1_MIN, LIMIT_J1_MAX, NUM_POINTS)
j2_samples = np.random.uniform(LIMIT_J2_MIN, LIMIT_J2_MAX, NUM_POINTS)
j3_samples = np.random.uniform(LIMIT_J3_MIN, LIMIT_J3_MAX, NUM_POINTS)

# 計算對應的座標
xs, ys, zs = forward_kinematics_np(j1_samples, j2_samples, j3_samples)

print(f"已生成 {NUM_POINTS} 個可達點。正在繪圖...")

# ==========================================
# 4. 3D 繪圖
# ==========================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 繪製點雲 (使用 Z 軸高度作為顏色映射)
scatter = ax.scatter(xs, ys, zs, c=zs, cmap='viridis', s=2, alpha=0.6, label='Reachable Points')

# 繪製機器人基座原點 (0,0,0)
ax.scatter([0], [0], [0], c='red', s=100, marker='X', label='Base Origin')

# 加入顏色條說明高度
cbar = plt.colorbar(scatter, ax=ax, shrink=0.6)
cbar.set_label('Height Z (mm)')

# 設定座標軸標籤
ax.set_xlabel('X Axis (Front/Back) [mm]')
ax.set_ylabel('Y Axis (Left/Right) [mm]')
ax.set_zlabel('Z Axis (Up/Down) [mm]')
ax.set_title(f'Robot Arm Workspace Visualization\n(Based on Software Limits)')

# 設定座標軸比例一致，這樣圖形才不會變形
# 找出最大的範圍
max_range = np.array([xs.max()-xs.min(), ys.max()-ys.min(), zs.max()-zs.min()]).max() / 2.0
mid_x = (xs.max()+xs.min()) * 0.5
mid_y = (ys.max()+ys.min()) * 0.5
mid_z = (zs.max()+zs.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.legend()
print("繪圖完成！請在彈出的視窗中旋轉觀察。")
plt.show()