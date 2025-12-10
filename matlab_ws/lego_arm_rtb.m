% [裝置: PC/Mac]
% 腳本: lego_arm_rtb_fix.m
% 修正 teach 報錯問題
clear; clc; close all;

% 檢查工具箱
if exist('Link', 'file') ~= 2
    error('錯誤: 找不到 Link 指令。請確認 Robotics Toolbox 路徑是否正確加入。');
end

%% 1. 參數設定
deg = pi/180;
mm = 1e-3;

% 關節限制
q_lims = [
    -135 135; 
    -29  29; 
    -38  30; 
    -180 180; 
    -90  90
] * deg;

%% 2. 定義連桿
% [L1] Base -> J1
L(1) = Link('d', 105*mm, 'a', 23*mm, 'alpha', 90*deg, 'standard');
L(1).qlim = q_lims(1,:);

% [L2] J1 -> J2
L(2) = Link('d', 0, 'a', 135*mm, 'alpha', 0, 'offset', 90*deg, 'standard');
L(2).qlim = q_lims(2,:);

% [L3] J2 -> J3
L(3) = Link('d', 0, 'a', 35*mm, 'alpha', 0, 'offset', -90*deg, 'standard');
L(3).qlim = q_lims(3,:);

% [L4] J3 -> J4
L(4) = Link('d', 0, 'a', 137*mm, 'alpha', -90*deg, 'standard');
L(4).qlim = q_lims(4,:);

% [L5] J4 -> J5
L(5) = Link('d', 97*mm, 'a', 0, 'alpha', 0, 'standard');
L(5).qlim = q_lims(5,:);

%% 3. 建立機器人
bot = SerialLink(L, 'name', 'LEGO 5-Axis');
disp('--- 機器人建立成功 ---');
bot.display();

%% 4. 啟動 Teach Pendant
disp('正在開啟模擬視窗...');

% [修正點] 手動給予初始角度 [0 0 0 0 0]
init_q = zeros(1, 5); 

% 啟動 GUI
bot.teach(init_q, 'rpy/zyx', 'view', [135 30]);