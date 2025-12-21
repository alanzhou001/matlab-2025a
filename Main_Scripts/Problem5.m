% =========================================================================
% 第五题：自适应多起点搜索 (Adaptive Multi-Start Search) - 5机协同干扰
% 特色：自适应控制参数 + 动态种群规模 + 混合优化
% =========================================================================
clear; clc; close all;
% addpath('Optimization_Functions'); % 请确保这些路径存在
% addpath('Models');

% --- 1. 全局场景参数定义 ---
GlobalParams.T_Base = [0, 200, 0];
GlobalParams.R_T = 7;
GlobalParams.H_T = 10;
GlobalParams.T_Bottom = [0, 200, 0]; 
GlobalParams.Target_Fake = [0, 0, 0];

% 导弹群 M1-M3
GlobalParams.Missiles(1).Pos0 = [20000, 0, 2000];
GlobalParams.Missiles(2).Pos0 = [19000, 600, 2100];
GlobalParams.Missiles(3).Pos0 = [18000, -600, 1900];
GlobalParams.V_M = 300; 

% 无人机群 FY1-FY5
GlobalParams.UAVs(1).Pos0 = [17800, 0, 1800];
GlobalParams.UAVs(2).Pos0 = [12000, 1400, 1400];
GlobalParams.UAVs(3).Pos0 = [6000, -3000, 700];
GlobalParams.UAVs(4).Pos0 = [11000, 2000, 1800];
GlobalParams.UAVs(5).Pos0 = [13000, -2000, 1300];
GlobalParams.V_Sink = 3;
GlobalParams.R_Smoke = 10;
GlobalParams.Smoke_Duration = 20;

% --- 2. 变量与边界定义 ---
Num_UAVs = 5;
Vars_Per_UAV = 8;
Total_Vars = Num_UAVs * Vars_Per_UAV;

lb_one = [0, 70, 0,  1,  1,  0, 0, 0];
ub_one = [2*pi, 140, 50, 10, 10, 10, 10, 10];
lb = repmat(lb_one, 1, Num_UAVs);
ub = repmat(ub_one, 1, Num_UAVs);

% --- 3. 多起点自动调参设置 ---

Num_Restarts = 100;  
Results_Log = struct('RunID', {}, 'Fval', {}, 'X', {}, 'SwarmSize', {});
fprintf('>>> 启动自适应多起点搜索 (共 %d 次任务)...\n', Num_Restarts);
fprintf('------------------------------------------------------------\n');

% 使用 DataQueue 实现并行环境下的进度显示
dq = parallel.pool.DataQueue;
afterEach(dq, @(varargin) fprintf('.'));
tick = 0;
startTime = tic;

% 定义目标函数
Fun_Obj = @(x) Obj_Problem5_MultiTarget(x, GlobalParams);

% 使用并行计算
parfor run_idx = 1:Num_Restarts
    % --- 动态调参：随 RunID 增加搜索强度 ---
    % 早期快速扫描解空间，后期深度挖掘
    current_SwarmSize = 60 + (run_idx-1) * 15; 
    current_MaxIter = 40 + (run_idx-1) * 5;
    
    % Step A: 启发式随机初始化
    init_swarm = Heuristic_Init_Randomized(current_SwarmSize, Total_Vars, GlobalParams, lb, ub);
    
    % Step B: 调用自适应 PSO (内部自动调整 w, c1, c2)
    pso_opts = struct('SwarmSize', current_SwarmSize, 'MaxIter', current_MaxIter, 'InitialSwarm', init_swarm);
    [x_pso, fval_pso] = run_adaptive_pso_silent(Fun_Obj, Total_Vars, lb, ub, pso_opts);
    
    % Step C: SQP 局部精修
    sqp_opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', 'MaxIterations', 40);
    try
        [x_sqp, fval_sqp] = fmincon(Fun_Obj, x_pso, [], [], [], [], lb, ub, [], sqp_opts);
    catch
        x_sqp = x_pso; fval_sqp = fval_pso;
    end
    
    % 存储结果
    Results_Log(run_idx).RunID = run_idx;
    Results_Log(run_idx).Fval = fval_sqp;
    Results_Log(run_idx).X = x_sqp;
    Results_Log(run_idx).SwarmSize = current_SwarmSize;

    % 向主线程发送进度信号
    send(dq, run_idx);
end

% --- 4. 统计与分析 ---
Best_Global_Val = inf;
Best_Global_X = [];
for k = 1:Num_Restarts
    if Results_Log(k).Fval < Best_Global_Val
        Best_Global_Val = Results_Log(k).Fval;
        Best_Global_X = Results_Log(k).X;
    end
    fprintf('任务 %d (规模 %d): 遮蔽时长 %.2f s\n', ...
        Results_Log(k).RunID, Results_Log(k).SwarmSize, -Results_Log(k).Fval);
end

fprintf('------------------------------------------------------------\n');
fprintf('最终最优遮蔽时长: %.4f 秒\n', -Best_Global_Val);

% 输出与保存
Output_Results(Best_Global_X, Num_UAVs);
save('Result_Problem5_Adaptive.mat', 'Best_Global_X', 'Best_Global_Val', 'Results_Log');

% =========================================================
% 自适应 PSO 函数 (自动调参核心)
% =========================================================
function [gbest, gbest_val] = run_adaptive_pso_silent(fun, nvars, lb, ub, opts)
    S = opts.SwarmSize;
    MaxIter = opts.MaxIter;
    pos = opts.InitialSwarm;
    vel = zeros(S, nvars);
    pbest_pos = pos;
    pbest_val = inf(S, 1);
    gbest_val = inf;
    gbest_pos = zeros(1, nvars);
    vmax = 0.15 * (ub - lb);

    for iter = 1:MaxIter
        % --- 自动参数调整逻辑 ---
        % 1. 非线性惯性权重: 初期大以便逃离局部最优，后期小以便精确收敛
        w = 0.9 - (0.9 - 0.4) * (iter / MaxIter)^2;
        
        % 2. 时变加速度常数: c1(个体经验) 递减，c2(群体经验) 递增
        c1 = 2.5 - 2.0 * (iter / MaxIter);
        c2 = 0.5 + 2.0 * (iter / MaxIter);

        for i = 1:S
            % 边界限制
            pos(i,:) = max(min(pos(i,:), ub), lb);
            val = fun(pos(i,:));
            
            if val < pbest_val(i)
                pbest_val(i) = val;
                pbest_pos(i,:) = pos(i,:);
            end
            if val < gbest_val
                gbest_val = val;
                gbest_pos = pos(i,:);
            end
        end
        
        % 速度更新
        r1 = rand(S, nvars); r2 = rand(S, nvars);
        vel = w*vel + c1*r1.*(pbest_pos - pos) + c2*r2.*(gbest_pos - pos);
        vel = max(min(vel, vmax), -vmax);
        pos = pos + vel;
    end
    gbest = gbest_pos;
end

% =========================================================
% 辅助输出函数
% =========================================================
function Output_Results(x, Num_UAVs)
    fprintf('%-8s %-8s %-8s | %-18s | %-18s\n', 'UAV_ID', 'Angle(°)', 'Vel(m/s)', 'Drop_Time(s)', 'Delay_Tau(s)');
    fprintf('--------------------------------------------------------------------------------\n');
    for i = 1:Num_UAVs
        idx = (i-1)*8 + 1;
        u = x(idx : idx+7);
        
        % 变量解析
        theta = rad2deg(u(1));
        v = u(2);
        % 投放时间：t1, t2 = t1+dt1, t3 = t2+dt2
        t_drops = [u(3), u(3)+u(4), u(3)+u(4)+u(5)];
        % 延时起爆：tau1, tau2, tau3
        taus = [u(6), u(7), u(8)];
        
        fprintf('FY-%-5d %-8.1f %-8.1f | [%.1f, %.1f, %.1f] | [%.1f, %.1f, %.1f]\n', ...
            i, theta, v, t_drops(1), t_drops(2), t_drops(3), taus(1), taus(2), taus(3));
    end
    fprintf('--------------------------------------------------------------------------------\n');
end