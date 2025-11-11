% Problem1_Solver_Precise.m
% 目的: 解决问题一，使用精确的 is_FullyShielded 函数计算单弹对 M1 的有效遮蔽时长。

clear; clc;

% --- 1. 常量定义 ---
V_M = 300;                  % 导弹速度 (m/s)
V_sink = 3;                 % 烟幕下沉速度 (m/s)
R_eff = 10;                 % 烟幕有效半径 (m)
Delta_T_eff = 20;           % 烟幕有效持续时间 (s)

P_O = [0, 0, 0];            % 假目标/原点
CB = [0, 200, 0];           % 真目标底面中心
RT = 7;                     % 真目标半径 (m)
HT = 10;                    % 真目标高度 (m)

M1_P0 = [20000, 0, 2000];   
FY1_P0 = [17800, 0, 1800];  
V_UAV_mag = 120;            

t_start = 0;
t_drop = t_start + 1.5;     
Delta_t_det = 3.6;
t_det = t_drop + Delta_t_det; % 5.1 s

% --- 2. 投放模型计算 ---
V_UAV = [-V_UAV_mag, 0, 0];

P_drop = FY1_P0 + V_UAV * t_drop;
P_det = get_SmokeDetPos(P_drop, t_drop, V_UAV, t_det); % 调用 get_SmokeDetPos

t_start_eff = t_det;
t_end_eff = t_det + Delta_T_eff; % 25.1 s

% --- 3. 迭代计算遮蔽时长 (使用 is_FullyShielded) ---
dt = 0.1; % 增大时间步长以保证运行速度 (0.5秒)
Total_Shielded_Time = 0.01;
t = t_start;

T_max_mission = 70; 

fprintf('--- 问题一：单弹精确遮蔽时长计算 (MultiStart) ---\n');
fprintf('烟幕有效时间窗: [%.2f s, %.2f s]\n', t_start_eff, t_end_eff);
fprintf('注意: 使用 dt=%.1fs 求解，运行时间较长。\n', dt);
fprintf('---------------------------------------------------\n');

% 循环只在烟幕有效的时间段内进行
t = t_start_eff;

while t <= t_end_eff && t <= T_max_mission
    
    PM_t = get_MissilePos(M1_P0, t, V_M, P_O); % 调用 get_MissilePos
    C_t = get_Smokecenter(P_det, t_det, t, V_sink); % 调用 get_Smokecenter
    
    % C_list 仅包含此刻有效的烟幕中心（此处仅一个）
    C_list_valid = C_t; 
    
    % 调用精确的判定函数 is_FullyShielded
    is_shielded = is_FullyShielded(PM_t, C_list_valid, R_eff, RT, HT, CB);
    
    if is_shielded
        Total_Shielded_Time = Total_Shielded_Time + dt;
        fprintf('t=%.2fs: 遮蔽成功。\n', t);
    else
        fprintf('t=%.2fs: 存在突破点。\n', t);
    end
    
    t = t + dt;
end

fprintf('---------------------------------------------------\n');
fprintf('*** 最终有效遮蔽时长 (精确模型): %.2f 秒 ***\n', Total_Shielded_Time);
fprintf('---------------------------------------------------\n');