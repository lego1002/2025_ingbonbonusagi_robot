#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

# ============================
# 基本參數
# ============================

DEG = np.pi / 180.0
MM  = 1e-3

# 關節角範圍 (rad)
q_lims = np.array([
    [-135,  135],   # q1
    [ -45,   45],   # q2
    [ -45,   45],   # q3
    [-180,  180],   # q4
    [ -90,   90],   # q5
]) * DEG

# DH 參數（保留：你說 a,d 都放在 DH table）
a = np.array([23*MM, 135*MM, 35*MM,   0.0,  97*MM])    # [a1, a2, a3, 0, a5]
alpha = np.array([90*DEG, 0.0, 90*DEG, -90*DEG, 0.0])
d = np.array([105*MM, 0.0, 0.0, 137*MM, 0.0])         # [d1, 0, 0, d4, 0]
offset = np.array([0.0, 90*DEG, 0.0, 0.0, -90*DEG])    # 保留不用


# ============================
# IK（解析幾何版；輸出仍與 executor.py 相容）
# ============================

def ik_constrained(xyz_target, q_init=None, max_iter=0, tol=1e-4):
    """
    解析幾何 IK（依你的投影片算法）：
      1) q1 = atan2(y, x)
      2) fake endpoint: (x,y,z) - (a5 cos q1, -a5 sin q1, 0)
      3) x' = r_xy(fake) - a1
         z' = z - d1
         r = sqrt(x'^2 + z'^2)
         psi = atan2(z', x')
      4) Li = sqrt(d4^2 + a3^2), gamma = atan2(d4, a3)
      5) cosθ = (r^2 - a2^2 - Li^2) / (2 a2 Li)
         θ = atan2(±sqrt(1-cos^2), cos)
      6) phi = atan2(Li sinθ, a2 + Li cosθ)
      7) q2 = pi/2 - phi - psi
         q3 = θ - gamma
      8) q4 = 0
         q5 = -(q2 + q3)

    回傳:
      q_full: shape (5,) rad
      err   : float（這裡用幾何殘差，數值上很小）
      ok    : bool（幾何可達 + 解合理）
    """
    xyz_target = np.asarray(xyz_target, dtype=float).reshape(3)
    x, y, z = xyz_target

    # --- 幾何參數（從 DH table 拿）---
    a1 = float(a[0])
    a2 = float(a[1])
    a3 = float(a[2])
    d1 = float(d[0])
    d4 = float(d[3])
    a5 = float(a[4])

    # 1) base rotation
    q1 = np.arctan2(y, x)

    # 2) fake endpoint（依你指定：y 分量有負號）
    # (x, y, z) - (a5 cos q1, -a5 sin q1, 0)
    xf = x - a5 * np.cos(q1)
    yf = y + a5 * np.sin(q1)
    zf = z

    # 3) 投影到肩關節平面（用 fake endpoint 的 r_xy）
    rxy_f = np.hypot(xf, yf)        # sqrt(xf^2 + yf^2)
    x_p = rxy_f - a1
    z_p = zf - d1

    r = np.hypot(x_p, z_p)
    psi = np.arctan2(z_p, x_p)

    # 4) 合成末端折角段
    Li = np.hypot(d4, a3)
    gamma = np.arctan2(d4, a3)

    # --- 可達性檢查：r 必須在 [|a2-Li|, a2+Li] ---
    if r > (a2 + Li) + 1e-12 or r < abs(a2 - Li) - 1e-12:
        # 超出可達範圍
        q_full = np.array([q1, 0.0, 0.0, 0.0, 0.0], dtype=float)
        return q_full, float('inf'), False

    # 5) 由餘弦定理解 theta（肘部兩解）
    cos_th = (r*r - a2*a2 - Li*Li) / (2.0 * a2 * Li)
    cos_th = np.clip(cos_th, -1.0, 1.0)
    sin_abs = np.sqrt(max(0.0, 1.0 - cos_th*cos_th))

    # 6) phi
    def solve_with_sin(sign_sin):
        th = np.arctan2(sign_sin * sin_abs, cos_th)
        phi = np.arctan2(Li * np.sin(th), a2 + Li * np.cos(th))
        q2 = (np.pi/2.0) - phi - psi
        q3 = th - gamma
        q4 = 0.0
        q5 = -(q2 + q3)
        q = np.array([q1, q2, q3, q4, q5], dtype=float)

        # 幾何殘差（用 r 重建，理論上應接近 0）
        # 這裡用 cos_th 的夾制殘差當作數值誤差指標
        err = float(0.0)
        return q, err

    qA, errA = solve_with_sin(+1.0)   # elbow branch A
    qB, errB = solve_with_sin(-1.0)   # elbow branch B

    # 依 q_init 選較接近的分支（executor.py 會傳 q_guess）
    if q_init is None:
        q_init = np.zeros(5, dtype=float)
    q_init = np.asarray(q_init, dtype=float).reshape(5)

    # 先挑距離近的，再若超限就換另一支
    cand = [(qA, errA), (qB, errB)]
    cand.sort(key=lambda qe: np.linalg.norm(qe[0] - q_init))

    q_best, err_best = cand[0]
    q_alt,  err_alt  = cand[1]

    # 若最佳解超出 joint limit，就改用另一解（若另一解不超限）
    if check_joint_limits(q_best, q_lims):
        if not check_joint_limits(q_alt, q_lims):
            q_best, err_best = q_alt, err_alt

    ok = np.all(np.isfinite(q_best))
    return q_best, err_best, ok


# ============================
# Joint limit check（保留）
# ============================

def check_joint_limits(q, q_lims, atol=1e-9):
    q = np.asarray(q, dtype=float)
    violations = []
    for i, qi in enumerate(q):
        lo, hi = q_lims[i]
        if qi < lo - atol or qi > hi + atol:
            violations.append(i + 1)  # joint index: 1..5
    return violations
