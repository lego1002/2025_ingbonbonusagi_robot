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
    [ -30,   30],   # q2
    [ -35,   35],   # q3
    [-180,  180],   # q4
    [ -90,   90],   # q5
]) * DEG

# DH 參數
a = np.array([23*MM, 135*MM, 35*MM,   0.0,  97*MM])
alpha = np.array([90*DEG, 0.0, 90*DEG, -90*DEG, 0.0])
d = np.array([105*MM, 0.0, 0.0, 137*MM, 0.0])
offset = np.array([0.0, 90*DEG, 0.0, 0.0, -90*DEG])

# ============================
# FK
# ============================

def dh_transform(a_i, alpha_i, d_i, theta_i):
    ca, sa = np.cos(alpha_i), np.sin(alpha_i)
    ct, st = np.cos(theta_i), np.sin(theta_i)

    return np.array([
        [ct, -st*ca,  st*sa, a_i*ct],
        [st,  ct*ca, -ct*sa, a_i*st],
        [0.0,   sa,     ca,    d_i],
        [0.0,  0.0,    0.0,   1.0],
    ])


def fkine(q):
    T = np.eye(4)
    for i in range(5):
        theta = q[i] + offset[i]
        T = T @ dh_transform(a[i], alpha[i], d[i], theta)
    return T


def end_effector_pos(q):
    T = fkine(q)
    return T[0:3, 3]

# ============================
# Jacobian (numeric)
# ============================

def numeric_jacobian_pos(q, eps=1e-6):
    J = np.zeros((3, 5))
    p0 = end_effector_pos(q)

    for j in range(5):
        dq = q.copy()
        dq[j] += eps
        p1 = end_effector_pos(dq)
        J[:, j] = (p1 - p0) / eps

    return J

# ============================
# IK（只用 q1~q3，約束 q4,q5）
# ============================

def ik_constrained(xyz_target, q_init=None, max_iter=80, tol=1e-4):
    xyz_target = np.asarray(xyz_target).reshape(3)

    if q_init is None:
        q_init = np.zeros(5)

    q_red = q_init[0:3].copy()

    S = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [0, 0, 0],
        [0,-1,-1],
    ], dtype=float)

    for _ in range(max_iter):
        q_full = np.zeros(5)
        q_full[0:3] = q_red
        q_full[4] = -(q_red[1] + q_red[2])

        e = xyz_target - end_effector_pos(q_full)
        if np.linalg.norm(e) < tol:
            return q_full, np.linalg.norm(e), True

        J = numeric_jacobian_pos(q_full) @ S
        dq = np.linalg.pinv(J) @ e
        q_red += dq

    return q_full, np.linalg.norm(e), False

# ============================
# Joint limit check
# ============================

def check_joint_limits(q, q_lims, atol=1e-9):
    q = np.asarray(q)
    violations = []

    for i, qi in enumerate(q):
        lo, hi = q_lims[i]
        if qi < lo - atol or qi > hi + atol:
            violations.append(i + 1)

    return violations
