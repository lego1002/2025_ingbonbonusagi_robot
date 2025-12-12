import numpy as np

# ============================
# 基本參數
# ============================

DEG = np.pi / 180.0
MM  = 1e-3

# 關節角範圍 (跟你 MATLAB 裡一樣，注意單位轉成 rad)
q_lims = np.array([
    [-135,  135],   # q1
    [ -29,   29],   # q2
    [ -30,   38],   # q3
    [-180,  180],   # q4
    [ -90,   90],   # q5
]) * DEG

# DH 參數 (standard DH)，單位用 SI (m, rad)
a = np.array([23*MM, 135*MM, 35*MM,   0.0,  97*MM])
alpha = np.array([90*DEG, 0.0, 90*DEG, -90*DEG, 0.0])
d = np.array([105*MM, 0.0, 0.0, 137*MM, 0.0])
offset = np.array([0.0, 90*DEG, 0.0, 0.0, -90*DEG])  # 這裡先照實際機構擺


# ============================
# DH 變換 & FK
# ============================

def dh_transform(a_i, alpha_i, d_i, theta_i):
    """標準 DH：給一組 (a, alpha, d, theta) 回傳 4x4 變換矩陣"""
    ca, sa = np.cos(alpha_i), np.sin(alpha_i)
    ct, st = np.cos(theta_i), np.sin(theta_i)

    return np.array([
        [ct, -st*ca,  st*sa, a_i*ct],
        [st,  ct*ca, -ct*sa, a_i*st],
        [0.0,   sa,     ca,    d_i],
        [0.0,  0.0,    0.0,   1.0],
    ])


def fkine(q):
    """
    Forward kinematics
    q: shape (5,) 關節角 (rad)
    回傳 4x4 齊次變換矩陣 T_0_5
    """
    T = np.eye(4)
    for i in range(5):
        theta = q[i] + offset[i]     # standard DH：theta = q + offset
        A_i = dh_transform(a[i], alpha[i], d[i], theta)
        T = T @ A_i
    return T


def end_effector_pos(q):
    """只拿末端位置 (x,y,z)，shape (3,)"""
    T = fkine(q)
    return T[0:3, 3]


# ============================
# 數值 Jacobian (只要位置部分 3x5)
# ============================

def numeric_jacobian_pos(q, eps=1e-6):
    """
    用數值微分計算位置 Jacobian:
      Jp[i,j] = d p_i / d q_j
    q: shape (5,)
    回傳 Jp: shape (3,5)
    """
    J = np.zeros((3, 5))
    p0 = end_effector_pos(q)

    for j in range(5):
        q_pert = q.copy()
        q_pert[j] += eps
        p1 = end_effector_pos(q_pert)
        J[:, j] = (p1 - p0) / eps

    return J


# ============================
# IK：只用 q1,q2,q3，約束 q4,q5
# ============================

def ik_constrained(xyz_target,
                   q_init=None,
                   max_iter=80,
                   tol=1e-4):
    """
    數值 IK：
      - 真正變數只有 q1,q2,q3 (reduced DOF)
      - q4 永遠 = 0
      - q5 永遠 = -(q2 + q3)   (offset 先當 0)

    xyz_target: 目標位置 [x,y,z] (m)
    q_init    : 初始猜測 (5,)，若 None 則用全 0

    回傳：
      q_full   : shape (5,) 的解
      err      : 最終位置誤差 (m)
      success  : True/False
    """
    xyz_target = np.asarray(xyz_target).reshape(3)

    if q_init is None:
        q_init = np.zeros(5)
    q_init = np.asarray(q_init).reshape(5)

    # 以目前姿態的 q1~q3 當初始值
    q_red = q_init[0:3].copy()   # [q1,q2,q3]

    # 約束矩陣 S：dq_full = S @ dq_red
    # dq1 = dq1
    # dq2 = dq2
    # dq3 = dq3
    # dq4 = 0
    # dq5 = -(dq2 + dq3)
    S = np.array([
        [1.0,  0.0,  0.0],
        [0.0,  1.0,  0.0],
        [0.0,  0.0,  1.0],
        [0.0,  0.0,  0.0],
        [0.0, -1.0, -1.0],
    ])

    for k in range(max_iter):
        # 從 q_red 組出完整的 5 軸角度
        q_full = np.zeros(5)
        q_full[0] = q_red[0]          # q1
        q_full[1] = q_red[1]          # q2
        q_full[2] = q_red[2]          # q3
        q_full[3] = 0.0               # q4 固定
        q_full[4] = -(q_red[1] + q_red[2])  # q5 = -(q2+q3)，offset 暫時視為 0

        # 末端位置 & 誤差
        p_now = end_effector_pos(q_full)
        e = xyz_target - p_now           # 3x1

        if np.linalg.norm(e) < tol:
            break

        # 數值 Jacobian (3x5)，再乘 S → 3x3 有約束 Jacobian
        Jp = numeric_jacobian_pos(q_full)   # 3x5
        J_eff = Jp @ S                      # 3x3

        # pseudo-inverse 解 dq_red
        dq_red = np.linalg.pinv(J_eff) @ e  # 3x1
        q_red = q_red + dq_red

    # 迭代結束，再組最後的 q_full
    q_full = np.zeros(5)
    q_full[0] = q_red[0]
    q_full[1] = q_red[1]
    q_full[2] = q_red[2]
    q_full[3] = 0.0
    q_full[4] = -(q_red[1] + q_red[2])

    # 最終誤差
    p_final = end_effector_pos(q_full)
    err = float(np.linalg.norm(xyz_target - p_final))
    success = (err < tol)

    return q_full, err, success

def check_joint_limits(q, q_lims, atol=1e-9):
    """
    檢查 q 是否超出 q_lims。
    回傳 violations: list of dict
      每個 dict 內容：
        joint: int (1-based)
        q: float (rad)
        q_deg: float
        lo, hi: float (rad)
        lo_deg, hi_deg: float
        side: "low" or "high"
        exceed: float (rad, 正值表示超出多少)
        exceed_deg: float
    """
    q = np.asarray(q).reshape(-1)
    violations = []

    for i in range(len(q)):
        lo, hi = q_lims[i, 0], q_lims[i, 1]
        qi = q[i]

        if qi < lo - atol:
            exceed = lo - qi
            violations.append({
                "joint": i + 1,
                "q": qi, "q_deg": qi / DEG,
                "lo": lo, "hi": hi,
                "lo_deg": lo / DEG, "hi_deg": hi / DEG,
                "side": "low",
                "exceed": exceed, "exceed_deg": exceed / DEG
            })
        elif qi > hi + atol:
            exceed = qi - hi
            violations.append({
                "joint": i + 1,
                "q": qi, "q_deg": qi / DEG,
                "lo": lo, "hi": hi,
                "lo_deg": lo / DEG, "hi_deg": hi / DEG,
                "side": "high",
                "exceed": exceed, "exceed_deg": exceed / DEG
            })

    return violations


def pretty_print_solution(q_sol, err_m, p_target=None):
    q_deg = q_sol / DEG
    print("q_sol (deg):", np.array2string(q_deg, precision=6, suppress_small=False))
    print("position error (m):", f"{err_m:.12g}")
    p_now = end_effector_pos(q_sol)
    print("end-effector position (m):", np.array2string(p_now, precision=9))
    if p_target is not None:
        p_target = np.asarray(p_target).reshape(3)
        print("target position (m):     ", np.array2string(p_target, precision=9))


def report_limit_violations(violations):
    print("⚠️ Joint limit violated:")
    for v in violations:
        j = v["joint"]
        side = "below lower" if v["side"] == "low" else "above upper"
        print(
            f"  - J{j}: {v['q_deg']:.3f}° is {side} limit "
            f"[{v['lo_deg']:.3f}°, {v['hi_deg']:.3f}°], "
            f"exceed = {v['exceed_deg']:.3f}°"
        )

# ============================
# 簡單測試
# ============================

if __name__ == "__main__":
    target = np.array([0.30, 0.00, 0.20])  # 目標位置(m)
    q0 = np.zeros(5)

    q_sol, err, ok = ik_constrained(target, q0)

    print("IK success (position-only):", ok)
    pretty_print_solution(q_sol, err, p_target=target)

    violations = check_joint_limits(q_sol, q_lims)

    if violations:
        report_limit_violations(violations)
        print("=> This target may be unreachable within joint limits (given your constraints).")
    else:
        print("✅ All joints within limits.")
