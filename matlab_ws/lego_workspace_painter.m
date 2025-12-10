% [裝置: PC/Mac]
% 腳本: lego_3d_tracer.m
% 功能: 邊動邊畫點，即時顯示 3D 工作空間輪廓
clear; clc; close all;

% 檢查工具箱
if exist('Link', 'file') ~= 2
    error('錯誤: 找不到 Link 指令。');
end

%% 1. 建立機器人 (最終版)
deg = pi/180;
mm = 1e-3;
q_lims = [-135 135; -29 29; -38 30; -180 180; -90 90] * deg;

L(1) = Link('d', 105*mm, 'a', 23*mm, 'alpha', 90*deg, 'standard', 'qlim', q_lims(1,:));
L(2) = Link('d', 0, 'a', 135*mm, 'alpha', 0, 'offset', 90*deg, 'standard', 'qlim', q_lims(2,:));
L(3) = Link('d', 0, 'a', 35*mm, 'alpha', 0, 'offset', -90*deg, 'standard', 'qlim', q_lims(3,:));
L(4) = Link('d', 0, 'a', 137*mm, 'alpha', -90*deg, 'standard', 'qlim', q_lims(4,:));
L(5) = Link('d', 97*mm, 'a', 0, 'alpha', 0, 'standard', 'qlim', q_lims(5,:));

bot = SerialLink(L, 'name', '3D Tracer');

%% 2. 設計 3D 掃描軌跡 (立體版)
% 我們要畫出一個 "立體" 的邊界：
% 1. 底部外圈 -> 2. 底部內圈 -> 3. 爬升 -> 4. 頂部外圈

steps = 40; % 動作解析度 (點的密度)

% 姿勢定義
arm_max_low  = [29, 30, 0, 0] * deg;     % 手伸最長 (低)
arm_min_low  = [-29, -38, -90, 0] * deg; % 手縮最短 (低)
arm_max_high = [0, -30, 0, 0] * deg;     % 手伸最長 (高 - 舉手)
arm_min_high = [-29, -38, 0, 0] * deg;   % 手縮最短 (高)

% --- 軌跡生成 ---

% [階段1: 底部外圈] J1 -135 -> +135
t1 = [jtraj(-135*deg, 135*deg, steps), repmat(arm_max_low, steps, 1)];

% [階段2: 右側內縮] 手臂縮回來
t2 = jtraj([135*deg, arm_max_low], [135*deg, arm_min_low], 20);

% [階段3: 底部內圈] J1 +135 -> -135
t3 = [jtraj(135*deg, -135*deg, steps), repmat(arm_min_low, steps, 1)];

% [階段4: 左側爬升] 手臂舉高
t4 = jtraj([-135*deg, arm_min_low], [-135*deg, arm_max_high], 20);

% [階段5: 頂部外圈] J1 -135 -> +135 (高空)
t5 = [jtraj(-135*deg, 135*deg, steps), repmat(arm_max_high, steps, 1)];

% 合併所有軌跡
full_traj = [t1; t2; t3; t4; t5];

%% 3. 動畫與即時繪點 (Real-time Plotting)
figure('Color', [1 1 1], 'Name', 'Real-time 3D Tracing');
view(135, 30); % 設定 3D 視角
axis([-0.5 0.5 -0.5 0.5 0 0.6]); % 固定座標軸範圍
grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('正在繪製 3D 工作空間...');

% 先畫出機器人初始位置 (不要讓它消失)
bot.plot(full_traj(1,:), 'view', [135 30], 'delay', 0);

disp('開始掃描... 紅點代表末端經過的位置');

% --- 手動動畫迴圈 ---
for i = 1:size(full_traj, 1)
    q = full_traj(i,:);
    
    % 1. 更新機器人姿勢 (只更新圖形，不重新畫座標軸)
    bot.animate(q); 
    
    % 2. 計算末端點位置 (Forward Kinematics)
    T = bot.fkine(q);
    P = T.transl; % [x, y, z]
    
    % 3. 畫點 (plot3)
    % 'r.' = 紅色點點, MarkerSize 可調整點的大小
    plot3(P(1), P(2), P(3), 'r.', 'MarkerSize', 8);
    
    % 4. 強制刷新畫面 (讓動畫動起來)
    drawnow; 
    
    % 稍微暫停一下，不然跑太快看不清楚 (可拿掉)
    % pause(0.01); 
end

disp('掃描完成！這些紅點構成了機器人的 3D 邊界框。');
rotate3d on; % 允許用滑鼠旋轉觀察