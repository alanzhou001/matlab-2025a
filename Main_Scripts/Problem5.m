% =========================================================================
% 第五题：多起点搜索 (Multi-Start Search) - 5机协同干扰
% 策略：多次独立运行 [随机启发式初始化 -> PSO全局搜索 -> SQP局部精修]
% =========================================================================
clear; clc; close all;
addpath('Optimization_Functions'); 
addpath('Models');

% --- 1. 全局场景参数定义 (保持不变) ---
GlobalParams.T_Base = [0, 200, 0];
GlobalParams.R_T = 7;
GlobalParams.H_T = 10;
GlobalParams.T_Bottom = [0, 200, 0]; 
GlobalParams.Target_Fake = [0, 0, 0];

% 导弹群 M1, M2, M3
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

% --- 2. 变量定义 ---
Num_UAVs = 5;
Vars_Per_UAV = 8;
Total_Vars = Num_UAVs * Vars_Per_UAV;

% 下界 lb 和 上界 ub
lb_one = [0,    70,  0,  1,  1,  0, 0, 0];
ub_one = [2*pi, 140, 50, 10, 10, 10, 10, 10];
lb = repmat(lb_one, 1, Num_UAVs);
ub = repmat(ub_one, 1, Num_UAVs);

% --- 3. 多起点搜索设置 ---
Num_Restarts = 10;  % 设置重启次数 (建议 5-10 次)
Best_Global_Val = inf; % 记录全局最小值 (负值)
Best_Global_X = [];

Results_Log = struct('RunID', {}, 'Fval', {}, 'X', {}); % 记录每次结果

fprintf('>>> 启动多起点搜索 (共 %d 次重启)...\n', Num_Restarts);
fprintf('------------------------------------------------------------\n');

% 如果有并行工具箱，建议将 for 改为 parfor 以加速
parfor run_idx = 1:Num_Restarts
    fprintf('=== [Worker] 正在执行第 %d 任务 ===\n', run_idx);
    
    % --- Step A: 随机化启发式初始化 ---
    % 注意：在 parfor 中使用 rand 是安全的，MATLAB 会自动处理随机流
    this_SwarmSize = 80;
    Initial_Swarm = Heuristic_Init_Randomized(this_SwarmSize, Total_Vars, GlobalParams, lb, ub);
    
    % --- Step B: 改进 PSO 全局搜索 ---
    % 【关键修改】：使用全新的局部变量名 current_opts，而不是外部可能存在的 pso_options
    current_opts = struct(); 
    current_opts.SwarmSize = this_SwarmSize;
    current_opts.MaxIter = 50; 
    current_opts.InitialSwarm = Initial_Swarm;
    current_opts.w = 0.9; 
    current_opts.c1 = 1.5; 
    current_opts.c2 = 1.5;
    
    % 定义目标函数句柄（确保 GlobalParams 是广播变量，不要在循环内修改它）
    Fun_Obj = @(x) Obj_Problem5_MultiTarget(x, GlobalParams);
    
    % 运行 PSO (传入 current_opts)
    [x_pso, fval_pso, ~] = run_improved_pso_silent(Fun_Obj, Total_Vars, lb, ub, current_opts);
    
    % --- Step C: SQP 局部精修 ---
    %同样使用局部变量定义选项
    local_sqp_options = optimoptions('fmincon', 'Display', 'off', ...
        'Algorithm', 'sqp', 'MaxIterations', 30);
    
    x_sqp = [];
    fval_sqp = 0;
    
    try
        [x_sqp, fval_sqp] = fmincon(Fun_Obj, x_pso, [], [], [], [], lb, ub, [], local_sqp_options);
    catch
        x_sqp = x_pso; 
        fval_sqp = fval_pso;
    end
    
    % --- 记录本轮结果 ---
    % 注意：在 parfor 中不能直接打印到同一行或更新全局变量
    % 必须按索引存入切片变量 Results_Log
    Results_Log(run_idx).RunID = run_idx;
    Results_Log(run_idx).Fval = fval_sqp;
    Results_Log(run_idx).X = x_sqp;
    
    % 注意：parfor 内部无法直接更新 Best_Global_Val 这样的全局统计变量
    % 我们需要在循环结束后再统计
end

% --- 循环结束后统计全局最优 ---
Best_Global_Val = inf;
Best_Global_X = [];

for k = 1:Num_Restarts
    if Results_Log(k).Fval < Best_Global_Val
        Best_Global_Val = Results_Log(k).Fval;
        Best_Global_X = Results_Log(k).X;
    end
    fprintf('Run %d 结果: %.4f 秒\n', Results_Log(k).RunID, -Results_Log(k).Fval);
end

fprintf('------------------------------------------------------------\n');
fprintf('并行搜索结束。\n');
fprintf('全局最优遮蔽总时长: %.4f 秒\n', -Best_Global_Val);

% --- 4. 输出最优结果 ---
Output_Results(Best_Global_X, Num_UAVs);
save('Result_Problem5_MultiStart.mat', 'Best_Global_X', 'Best_Global_Val', 'Results_Log');

% 绘制各次运行结果对比
figure;
bar([Results_Log.RunID], -[Results_Log.Fval]);
xlabel('Run ID'); ylabel('Total Shielding Time (s)');
title('Comparison of Multi-Start Search Results');
grid on;

% =========================================================
% 辅助函数：输出详细结果
% =========================================================
function Output_Results(x, Num_UAVs)
    for i = 1:Num_UAVs
        idx = (i-1)*8 + 1;
        u_vars = x(idx : idx+7);
        theta = u_vars(1); v = u_vars(2);
        t1 = u_vars(3); t2 = t1 + u_vars(4); t3 = t2 + u_vars(5);
        tau = u_vars(6:8);
        
        fprintf('\n[无人机 FY%d] 最终策略:\n', i);
        fprintf('  航向: %.1f°, 速度: %.1f m/s\n', rad2deg(theta), v);
        fprintf('  投放时刻: [%.2f, %.2f, %.2f] s\n', t1, t2, t3);
        fprintf('  起爆延时: [%.2f, %.2f, %.2f] s\n', tau(1), tau(2), tau(3));
    end
end

function [gbest, gbest_val, loss_history] = run_improved_pso_silent(fun, nvars, lb, ub, options)
% 逻辑与之前完全一致，只是去掉了每一步的 fprintf
    SwarmSize = options.SwarmSize;
    MaxIter = options.MaxIter;
    
    if isfield(options, 'InitialSwarm') && ~isempty(options.InitialSwarm)
        pos = options.InitialSwarm;
    else
        pos = repmat(lb, SwarmSize, 1) + rand(SwarmSize, nvars) .* repmat(ub - lb, SwarmSize, 1);
    end
    
    vel = zeros(SwarmSize, nvars);
    pbest_pos = pos;
    pbest_val = inf(SwarmSize, 1);
    gbest_val = inf;
    gbest_pos = zeros(1, nvars);
    loss_history = [];
    vmax = 0.2 * (ub - lb);

    for iter = 1:MaxIter
        for i = 1:SwarmSize
            pos(i,:) = max(pos(i,:), lb);
            pos(i,:) = min(pos(i,:), ub);
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
        loss_history(iter) = gbest_val;
        
        w = options.w * (1 - iter/MaxIter);
        r1 = rand(SwarmSize, nvars);
        r2 = rand(SwarmSize, nvars);
        vel = w*vel + options.c1*r1.*(pbest_pos - pos) + options.c2*r2.*(gbest_pos - pos);
        
        vel = max(vel, -vmax);
        vel = min(vel, vmax);
        pos = pos + vel;
    end
    gbest = gbest_pos;
end