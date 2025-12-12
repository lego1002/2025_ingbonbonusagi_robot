% [裝置: PC/Mac]
% 腳本: lego_auto_validation.m
% 功能: 自動化動畫演示 + 即時終端機回報 (幾何/方向/IK)
clear; clc; close all;

if exist('Link', 'file') ~= 2
    error('錯誤: 找不到 Link 指令。');
end

%% 1. 建立機器人模型 (最終確認版)
deg = pi/180; mm = 1e-3;
q_lims = [-135 135; -29 29; -38 30; -180 180; -90 90] * deg;

% L1: Base -> J1
L(1) = Link('d', 105*mm, 'a', 23*mm, 'alpha', 90*deg, 'standard', 'qlim', q_lims(1,:));
% L2: Shoulder
L(2) = Link('d', 0, 'a', 135*mm, 'alpha', 0, 'offset', 90*deg, 'standard', 'qlim', q_lims(2,:));
% L3: Elbow (Z-offset 35mm)
L(3) = Link('d', 0, 'a', 35*mm, 'alpha', -90*deg, 'standard', 'qlim', q_lims(3,:));
% L4: Wrist Base (X-offset 137mm, Roll)
L(4) = Link('d', 137*mm, 'a', 0, 'alpha', 90*deg, 'standard', 'qlim', q_lims(4,:));
% L5: Gripper (X-offset 97mm, Yaw)
L(5) = Link('d', 0, 'a', 97*mm, 'alpha', 0, 'offset', 90*deg, 'standard', 'qlim', q_lims(5,:));

bot = SerialLink(L, 'name', 'LEGO AutoCheck');

% 初始化繪圖視窗
figure('Color', [1 1 1], 'Name', 'LEGO Validation Visualizer');
bot.plot(zeros(1,5), 'view', [135 30], 'workspace', [-0.2 0.5 -0.3 0.3 0 0.5]);
light; % 加個光影比較好看

disp('========================================');
disp('      樂高 5 軸機械臂 - 自動化驗收系統      ');
disp('========================================');
pause(1);

%% --- 階段 1: 幾何尺寸驗證 (歸零) ---
print_header('測試 1: 幾何尺寸驗證 (Ruler Test)');
disp('動作: 機器人移動至歸零位置 (Home Position)...');

% 動畫: 慢慢歸零
q_home = zeros(1,5);
bot.animate(q_home); 
drawnow; pause(0.5);

% 計算
T_home = bot.fkine(q_home);
pos = T_home.transl * 1000; % mm
expected_z = 105 + 135 + 35;
expected_x = 23 + 137 + 97;

% 回報
fprintf('  [檢查] Z軸高度 (預期 %.1f mm) ... 實測: %.4f mm\n', expected_z, pos(3));
fprintf('  [檢查] X軸延伸 (預期 %.1f mm) ... 實測: %.4f mm\n', expected_x, pos(1));

if abs(pos(3)-expected_z) < 0.1 && abs(pos(1)-expected_x) < 0.1
    print_result('PASS', '模型尺寸與硬體完全吻合');
else
    print_result('FAIL', '尺寸不符，請檢查 DH 參數');
end
pause(1);

%% --- 階段 2: 關鍵方向驗證 (Q4 Roll / Q5 Yaw) ---
print_header('測試 2: 關鍵軸向驗證 (Orientation)');
disp('動作: 檢查 Q5 旋轉軸是否朝上 (Yaw)...');

% 計算矩陣
T_total = zeros(4,4,5); T_current = eye(4);
for i = 1:5
    T_link = double(L(i).A(0));
    T_current = T_current * T_link;
    T_total(:,:,i) = T_current;
end

% 向量分析
z5_axis = T_total(1:3, 3, 5); % Q5 Z軸
base_z = [0; 0; 1];           % Base Z軸

fprintf('  [檢查] Q5 Z軸向量: [%.4f, %.4f, %.4f]\n', z5_axis);

if abs(dot(z5_axis, base_z) - 1) < 1e-4
    print_result('PASS', 'Q5 為 Yaw 構型 (平行 Base Z，水平旋轉)');
else
    print_result('FAIL', 'Q5 方向錯誤');
end
pause(1);

%% --- 階段 3: 極限運動演示 (Gymnastics) ---
print_header('測試 3: 極限運動演示 (Range of Motion)');
disp('動作: 執行 Q5 左右掃描 (Yaw Motion)...');

% 產生軌跡: Q5 從 -90 轉到 +90
traj_q5 = jtraj(q_home, [0 0 0 0 90*deg], 30);
traj_q5_back = jtraj([0 0 0 0 90*deg], [0 0 0 0 -90*deg], 60);

% 播放動畫
bot.plot(traj_q5);
bot.plot(traj_q5_back);
bot.animate(q_home); % 回正

disp('動作: 執行 Q4 滾轉測試 (Roll Motion)...');
% 產生軌跡: Q4 從 -90 轉到 +90
traj_q4 = jtraj(q_home, [0 0 0 90*deg 0], 30);
traj_q4_back = jtraj([0 0 0 90*deg 0], [0 0 0 -90*deg 0], 60);

% 播放動畫
bot.plot(traj_q4);
bot.plot(traj_q4_back);
bot.animate(q_home); % 回正

print_result('DONE', '運動演示完成，請確認夾爪動作是否符合預期');
pause(1);

%% --- 階段 4: 直線軌跡 IK 驗證 (Straight Line) ---
print_header('測試 4: 智能畫線測試 (Linear Path IK)');
disp('動作: 計算 IK 並在空中畫出 20cm 直線...');

% 定義起點與終點 (在前方 20cm 處畫一條橫線)
% 起點: x=200, y=-100, z=100
start_pos = transl(0.2, -0.1, 0.1) * rpy2tr(0, 0, 0); 
end_pos   = transl(0.2,  0.1, 0.1) * rpy2tr(0, 0, 0);

% 生成直線路徑 (Cartesian Trajectory)
T_traj = ctraj(start_pos, end_pos, 50);

% 解算 IK
mask = [1 1 1 1 1 0]; % 忽略最後一個 Yaw 旋轉約束
try
    q_traj = bot.ikine(T_traj, 'mask', mask, 'q0', q_home);
    
    % 繪製並畫出軌跡線
    hold on;
    % 畫出目標直線(綠色)
    plot3([0.2 0.2], [-0.1 0.1], [0.1 0.1], 'g--', 'LineWidth', 2);
    
    % 機器人開始畫(紅色點點)
    bot.plot(q_traj, 'trail', 'r.', 'fps', 40);
    
    print_result('PASS', '直線繪製成功，IK 解算正常');
catch
    print_result('FAIL', 'IK 解算失敗 (可能是目標點超出工作範圍)');
end

disp(' ');
disp('========================================');
disp('             測試結束 (End)             ');
disp('========================================');


%% 輔助函數 (美化輸出用)
function print_header(text)
    fprintf('\n----------------------------------------\n');
    fprintf('>>> %s\n', text);
    fprintf('----------------------------------------\n');
end

function print_result(status, msg)
    if strcmp(status, 'PASS')
        fprintf('  [結果] ✅ PASS: %s\n', msg);
    elseif strcmp(status, 'DONE')
        fprintf('  [結果] 🔵 DONE: %s\n', msg);
    else
        fprintf('  [結果] ❌ FAIL: %s\n', msg);
    end
end