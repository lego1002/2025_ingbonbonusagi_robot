import math

class LegoRobotIK:
    def __init__(self):
        # --- 1. 機械結構 ---
        self.d1 = 55.79   
        self.a1 = 23.91   
        self.a2 = 134.0   
        self.a3 = 186.0   
        
        # --- 2. 齒輪比 (原廠設定) ---
        self.GR_J1 = 7.0 
        self.GR_J2 = 70.0 / 9.0 
        self.GR_J3 = 5.0

        # --- 3. 方向修正 ---
        self.DIR_J1 = -1
        self.DIR_J2 = -1 
        self.DIR_J3 = 1

        # --- 4. L型歸零校正 ---
        self.OFFSET_J2 = 90.0
        self.OFFSET_J3 = -90.0
        
        # --- 5. 關節角度限制 ---
        self.LIMIT_J2_MIN = 90.0 - 22.0
        self.LIMIT_J2_MAX = 90.0 + 33.0
        self.LIMIT_J3_MIN = -90.0 - 25.0
        self.LIMIT_J3_MAX = -90.0 + 25.0

    def get_motor_instructions(self, x, y, z):
        """逆向運動學 (IK): 輸入座標 -> 輸出馬達角度"""
        # 1. J1 底座
        theta1_phys = math.degrees(math.atan2(y, x))
        
        # 2. 投影計算
        r_ground = math.sqrt(x**2 + y**2) - self.a1
        z_arm = z - self.d1
        reach = math.sqrt(r_ground**2 + z_arm**2)
        
        # 範圍修正
        max_len = self.a2 + self.a3
        if reach > max_len:
             scale = max_len / reach
             r_ground *= scale
             z_arm *= scale
             reach = max_len
             
        # 3. IK 解算
        try:
            alpha = math.degrees(math.atan2(z_arm, r_ground))
            
            cos_beta = (self.a2**2 + reach**2 - self.a3**2) / (2 * self.a2 * reach)
            cos_beta = max(-1.0, min(1.0, cos_beta))
            beta = math.degrees(math.acos(cos_beta))
            
            theta2_phys = alpha + beta 
            
            cos_gamma = (self.a2**2 + self.a3**2 - reach**2) / (2 * self.a2 * self.a3)
            cos_gamma = max(-1.0, min(1.0, cos_gamma))
            gamma = math.degrees(math.acos(cos_gamma))
            
            theta3_phys = -(180 - gamma) 
            
        except Exception as e:
            raise ValueError(f"數學運算失敗: {e}")

        # 4. 限位檢查
        if not (self.LIMIT_J2_MIN <= theta2_phys <= self.LIMIT_J2_MAX):
            raise ValueError(f"J2 超出極限! {theta2_phys:.1f} (範圍 {self.LIMIT_J2_MIN:.1f}~{self.LIMIT_J2_MAX:.1f})")
            
        if not (self.LIMIT_J3_MIN <= theta3_phys <= self.LIMIT_J3_MAX):
            raise ValueError(f"J3 超出極限! {theta3_phys:.1f} (範圍 {self.LIMIT_J3_MIN:.1f}~{self.LIMIT_J3_MAX:.1f})")

        # 5. 轉換指令
        m1 = theta1_phys * self.DIR_J1 * self.GR_J1
        m2 = (theta2_phys - self.OFFSET_J2) * self.DIR_J2 * self.GR_J2
        m3 = (theta3_phys - self.OFFSET_J3) * self.DIR_J3 * self.GR_J3
        
        return m1, m2, m3

    def forward_kinematics(self, j1_phys, j2_phys, j3_phys):
        """
        [新增] 正向運動學 (FK): 輸入物理角度 -> 輸出座標 (x, y, z)
        這能幫助我們算出歸零點的真實座標
        """
        # 角度轉弧度
        t1 = math.radians(j1_phys)
        t2 = math.radians(j2_phys)
        # 注意: 我們定義的 t3 是相對彎曲角，實際幾何角度是 t2 + t3
        t_sum = math.radians(j2_phys + j3_phys)
        
        # 計算 R (水平投影長度)
        # R = a1 + a2*cos(t2) + a3*cos(t2+t3)
        r = self.a1 + self.a2 * math.cos(t2) + self.a3 * math.cos(t_sum)
        
        # 計算座標
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        z = self.d1 + self.a2 * math.sin(t2) + self.a3 * math.sin(t_sum)
        
        return x, y, z