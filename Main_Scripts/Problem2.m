clear; clc; close all;

% --- 1. 参数设置 ---
% 导弹 M1 参数
GlobalParams.M0 = [20000, 0, 2000];   % M1 初始位置
GlobalParams.Target_Fake = [0, 0, 0]; % 假目标(导弹飞行朝向)
GlobalParams.V_M = 300;               % 导弹速度 m/s

% 真目标参数
GlobalParams.T_Bottom = [0, 200, 0];  % 真目标底部中心
GlobalParams.R_T = 7;                 % 真目标半径
GlobalParams.H_T = 10;                % 真目标高度

% 无人机 FY1 参数
GlobalParams.UAV_Pos0 = [17800, 0, 1800]; % FY1 初始位置
GlobalParams.V_Sink = 3;              % 烟雾下沉速度
GlobalParams.R_Smoke = 10;            % 烟雾半径
GlobalParams.Smoke_Duration = 20;     % 烟雾持续时间

% --- 2. 优化变量边界 ---
% 变量 x = [theta (rad), v_d (m/s), t_drop (s), tau (s)]
% 限制范围
lb = [0,       70,   0,   0];     % 下界
ub = [2*pi,    140,  60,  10];    % 上界

% --- 3. 粒子群优化 (手写版 PSO) 设置 ---
pso_options.SwarmSize = 50;       % 粒子数量
pso_options.MaxIter = 100;        % 最大迭代次数
pso_options.w = 0.8;              % 惯性权重
pso_options.c1 = 1.5;             % 个体学习因子
pso_options.c2 = 1.5;             % 群体学习因子

fprintf('正在启动自定义粒子群优化...\n');

% 定义目标函数 
ObjFcn = @(x) -Calculate_Shielding_Time(x, GlobalParams);

% 调用自定义 PSO 函数 
[x_opt, fval, loss_history] = run_simple_pso(ObjFcn, 4, lb, ub, pso_options);

% --- 4. 结果输出 ---
Max_Shield_Time = -fval; % 取反还原回正数

fprintf('\n================ 优化结果 ================\n');
fprintf('最优遮蔽时长: %.4f 秒\n', Max_Shield_Time);
fprintf('------------------------------------------\n');
fprintf('最优策略参数:\n');
fprintf('航向角 (theta):   %.2f 度 (%.4f rad)\n', rad2deg(x_opt(1)), x_opt(1));
fprintf('飞行速度 (v_d):    %.2f m/s\n', x_opt(2));
fprintf('投放时刻 (t_drop): %.4f s\n', x_opt(3));
fprintf('起爆延时 (tau):    %.4f s\n', x_opt(4));
fprintf('起爆时刻 (t_det):  %.4f s\n', x_opt(3) + x_opt(4));

% 绘制收敛曲线
figure;
plot(loss_history, 'LineWidth', 2);
xlabel('迭代次数'); ylabel('目标函数值 (负时长)');
title('PSO 收敛曲线');
grid on;