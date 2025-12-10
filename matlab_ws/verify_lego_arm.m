% 清除環境
clear; clc; close all;

% 單位轉換 helper
deg = pi/180;
mm = 1e-3;

%% 1. 定義連桿 (Links) - 使用 Standard DH
% 語法: Link('d', d, 'a', a, 'alpha', alpha, 'offset', theta_offset)
% 注意: Peter Corke 的 offset 就是我們 DH 表中的 theta 偏差值

% L1: Base -> J1 (a=23mm 位移, alpha=90度)
% 'standard' 參數代表使用標準 DH (預設值)
L(1) = Link('d', 105*mm, 'a', 23*mm, 'alpha', 90*deg, 'standard');
L(1).qlim = [-135 135] * deg; % 設定關節極限

% L2: J1 -> J2 (大臂 135mm, 直立 Offset 90度)
L(2) = Link('d', 0, 'a', 135*mm, 'alpha', 0, 'offset', 90*deg, 'standard');
L(2).qlim = [-29 29] * deg;

% L3: J2 -> J3 (小臂前段 35mm, 轉水平 Offset -90度)
L(3) = Link('d', 0, 'a', 35*mm, 'alpha', 0, 'offset', -90*deg, 'standard');
L(3).qlim = [-38 30] * deg;

% L4: J3 -> J4 (小臂後段 137mm, 手腕俯仰 alpha -90度)
% 注意: 這裡我們將 137mm 放在 'a' (手臂延伸方向)
L(4) = Link('d', 0, 'a', 137*mm, 'alpha', -90*deg, 'standard');
L(4).qlim = [-180 180] * deg;

% L5: J4 -> J5 (夾爪長度 97mm)
% 注意: 夾爪通常是沿著 Z 軸長出去的，所以用 'd'
L(5) = Link('d', 97*mm, 'a', 0, 'alpha', 0, 'standard');
L(5).qlim = [-90 90] * deg;

%% 2. 建立機器人物件 (SerialLink)
lego_bot = SerialLink(L, 'name', 'LEGO 5-Axis Arm');

% 顯示機器人參數表
disp('機器人 DH 參數表:');
lego_bot.display();

%% 3. 啟動強大的內建 GUI (Teach Pendant)
% 這行指令會直接跳出一個視窗，包含滑桿、座標顯示、姿態控制
disp('正在啟動模擬視窗...');
lego_bot.teach('rpy/zyx'); 

% 說明: 
% 'rpy/zyx' 代表在 GUI 中顯示 Roll-Pitch-Yaw 角度