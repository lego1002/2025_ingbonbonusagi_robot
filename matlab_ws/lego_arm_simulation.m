% [裝置: PC/Mac]
% 腳本: lego_arm_exact_final.m
% 修正重點:
% 1. L4 產生 d=137 的 X 向位移 (Z軸朝前)
% 2. L5 產生 a=97 的 X 向位移，且 Z 軸朝上 (平行 Base Z)
% 3. L5 增加 Offset=90 (讓初始位置正確指向前方)
clear; clc; close all;

if exist('Link', 'file') ~= 2
    error('錯誤: 找不到 Link 指令。');
end

%% 1. 參數設定
deg = pi/180; mm = 1e-3;
q_lims = [-135 135; -29 29; -38 30; -180 180; -90 90] * deg;

%% 2. 定義連桿 (Exact User Spec)

% [L1] Base -> J1
L(1) = Link('d', 105*mm, 'a', 23*mm, 'alpha', 90*deg, 'standard', 'qlim', q_lims(1,:));

% [L2] Shoulder
L(2) = Link('d', 0, 'a', 135*mm, 'alpha', 0, 'offset', 90*deg, 'standard', 'qlim', q_lims(2,:));

% [L3] Elbow -> Wrist Base
% a=35: 垂直 Z 位移 (這會在 Q3 座標系中表現為 X 軸方向的長度，因為 Q3 Z 軸是指向側邊的)
% alpha=-90: 關鍵！把 Z 軸從「側邊」轉向「前方」。
% 這樣下一軸 (L4) 的 d 參數就會變成 X 方向的位移。
L(3) = Link('d', 0, 'a', 35*mm, 'alpha', -90*deg, 'standard', 'qlim', q_lims(3,:));

% [L4] Wrist Base -> Q5 Base
% d=137: 因為上一軸 alpha=-90，現在 Z 軸朝前。
%        所以 d=137 代表沿著 Z 軸 (也就是世界座標 X 方向) 前進 137mm。
% alpha=90: [關鍵] 把 Z 軸從「前方」轉向「上方」。
%           這樣 Q5 的旋轉軸 (Z) 就會平行於基座 Z 軸。
L(4) = Link('d', 137*mm, 'a', 0, 'alpha', 90*deg, 'standard', 'qlim', q_lims(4,:));

% [L5] Q5 -> Gripper Tip
% a=97: 連桿長度。因為 Q5 Z 軸朝上，X 軸是在水平面。
% offset=90: [關鍵] 您的要求 "90度才是0度"。
%            這會把 X 軸從「側邊」轉向「前方」。
%            因此 a=97 就會變成沿著世界座標 X 方向延伸。
L(5) = Link('d', 0, 'a', 97*mm, 'alpha', 0, 'offset', 90*deg, 'standard', 'qlim', q_lims(5,:));

%% 3. 建立機器人
bot = SerialLink(L, 'name', 'LEGO Perfect Spec');
disp('--- 參數設定完成 ---');

%% 4. 嚴格幾何驗證
q_zero = zeros(1,5);

% 手動計算矩陣
T_total = zeros(4,4,5); T_current = eye(4);
for i = 1:5
    T_link = double(L(i).A(q_zero(i)));
    T_current = T_current * T_link;
    T_total(:,:,i) = T_current;
end

% --- 驗證 1: Q5 Z軸 是否朝上 (平行 Base Z) ---
z1_axis = [0; 0; 1]; % Base Z
z5_axis = T_total(1:3, 3, 5); % Q5 Z

fprintf('\n[驗證 1] Q5 Z軸方向:\n');
fprintf('Q5 Z-axis: [%.4f, %.4f, %.4f]\n', z5_axis);

if abs(dot(z1_axis, z5_axis) - 1) < 1e-4
    disp('✅ 通過：Q5 Z 軸朝上 (平行 Base Z)。');
else
    disp('❌ 失敗：Q5 Z 軸方向不對。');
end

% --- 驗證 2: 夾爪是否指向前方 (X 方向) ---
% L5 的 a=97 應該沿著世界座標 X 軸延伸
pos_4 = T_total(1:3, 4, 4);
pos_5 = T_total(1:3, 4, 5);
disp_vec = pos_5 - pos_4;

x_world = [1; 0; 0]; % World X

fprintf('\n[驗證 2] Q5 (a=97) 延伸方向:\n');
fprintf('延伸向量: [%.4f, %.4f, %.4f]\n', disp_vec);
fprintf('長度: %.4f mm\n', norm(disp_vec)*1000);

if abs(dot(disp_vec/norm(disp_vec), x_world) - 1) < 1e-4
    disp('✅ 通過：夾爪沿著 X 軸 (前方) 延伸。');
else
    disp('❌ 失敗：夾爪沒有指向前方。');
end

%% 5. 啟動模擬
disp(' ');
disp('啟動 GUI...');
% view 設為 [135 30]
bot.teach(q_zero, 'rpy/zyx', 'view', [135 30]);