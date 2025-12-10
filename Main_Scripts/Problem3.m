clear; clc;

% --- 常量定义 ---
V_M = 300;                  % 导弹速度 (m/s)
V_sink = 3;                 % 烟幕下沉速度 (m/s)
R_eff = 10;                 % 烟幕有效半径 (m)
Delta_T_eff = 20;           % 烟幕有效持续时间 (s)

P_O = [0, 0, 0];            % 假目标/原点
CB = [0, 200, 0];           % 真目标底面中心
RT = 7;                     % 真目标半径 (m)
HT = 10;                    % 真目标高度 (m)

M1_P0 = [20000, 0, 2000];   % 导弹M1初始位置
FY1_P0 = [17800, 0, 1800];  % 无人机FY1初始位置

% --- 优化参数设置 ---
num_iterations = 3;         % 迭代次数（您可以调整这个值）
Max_Iterations = 50;        
N_theta_divisions = 6; 
N_v_divisions = 3;          
N_t_drop_divisions = 4;     
N_tau_divisions = 3;   
max_points=2000; %最大粗网格数        

% --- 变量边界约束 ---
lb = [179.5*pi/180, 70, 0, 2, 4, 3, 3, 3];    
ub = [180.5*pi/180, 140, 2, 4, 6, 6, 6, 6]; 

% --- 线性不等式约束 (投放间隔 >= 1s) ---
A = [0, 0, 1, -1, 0, 0, 0, 0;     % t_drop2 - t_drop1 >= 1
     0, 0, 0, 1, -1, 0, 0, 0];    % t_drop3 - t_drop2 >= 1
b = [-1; -1];                      

% --- 初始化最优解 ---
best_x = [];
best_fval = inf;

% --- 迭代细化搜索 ---
for iter = 1:num_iterations
    fprintf('\n=== 第 %d 次迭代细化搜索 ===\n', iter);
    
    % 确定当前搜索范围
    if isempty(best_x)
        % 第一次迭代使用全局范围
        current_lb = lb;
        current_ub = ub;
    else
        % 后续迭代在最优解附近缩小范围
        range_scale = 0.5 ^ iter;  % 随迭代次数缩小范围
        
        current_lb = max(lb, best_x - (ub - lb) * range_scale);
        current_ub = min(ub, best_x + (ub - lb) * range_scale);
        
        % 确保投放时间间隔约束
        current_lb(4) = max(current_lb(4), current_lb(3) + 1);
        current_lb(5) = max(current_lb(5), current_lb(4) + 1);
        
        fprintf('当前搜索范围:\n');
        fprintf('  航向角: [%.2f°, %.2f°]\n', current_lb(1)*180/pi, current_ub(1)*180/pi);
        fprintf('  速度: [%.1f, %.1f] m/s\n', current_lb(2), current_ub(2));
        fprintf('  投放时间1: [%.1f, %.1f] s\n', current_lb(3), current_ub(3));
        fprintf('  投放时间2: [%.1f, %.1f] s\n', current_lb(4), current_ub(4));
        fprintf('  投放时间3: [%.1f, %.1f] s\n', current_lb(5), current_ub(5));
        fprintf('  起爆延时1: [%.1f, %.1f] s\n', current_lb(6), current_ub(6));
        fprintf('  起爆延时2: [%.1f, %.1f] s\n', current_lb(7), current_ub(7));
        fprintf('  起爆延时3: [%.1f, %.1f] s\n', current_lb(8), current_ub(8));
    end
    
    % --- 生成当前迭代的网格起点 ---
    fprintf('生成第 %d 次迭代的网格起点...\n', iter);
    
    theta_grid = linspace(current_lb(1), current_ub(1), N_theta_divisions);
    v_grid = linspace(current_lb(2), current_ub(2), N_v_divisions);
    t_drop1_grid = linspace(current_lb(3), current_ub(3), N_t_drop_divisions);
    t_drop2_grid = linspace(current_lb(4), current_ub(4), N_t_drop_divisions);
    t_drop3_grid = linspace(current_lb(5), current_ub(5), N_t_drop_divisions);
    tau1_grid = linspace(current_lb(6), current_ub(6), N_tau_divisions);
    tau2_grid = linspace(current_lb(7), current_ub(7), N_tau_divisions);
    tau3_grid = linspace(current_lb(8), current_ub(8), N_tau_divisions);
    
    % 生成满足间隔约束的起点
    start_points = [];
    point_count = 0;
    
    for i = 1:length(theta_grid)
        for j = 1:length(v_grid)
            for t1_idx = 1:length(t_drop1_grid)
                for t2_idx = 1:length(t_drop2_grid)
                    for t3_idx = 1:length(t_drop3_grid)
                        t1 = t_drop1_grid(t1_idx);
                        t2 = t_drop2_grid(t2_idx);
                        t3 = t_drop3_grid(t3_idx);
                        
                        % 检查投放时间间隔是否满足要求
                        if (t2 >= t1 + 1) && (t3 >= t2 + 1) && ...
                           (t1 >= current_lb(3)) && (t1 <= current_ub(3)) && ...
                           (t2 >= current_lb(4)) && (t2 <= current_ub(4)) && ...
                           (t3 >= current_lb(5)) && (t3 <= current_ub(5))
                            % 对于满足条件的投放时间，生成所有可能的起爆延时组合
                            for m1 = 1:length(tau1_grid)
                                for m2 = 1:length(tau2_grid)
                                    for m3 = 1:length(tau3_grid)
                                        tau1 = tau1_grid(m1);
                                        tau2 = tau2_grid(m2);
                                        tau3 = tau3_grid(m3);
                                        new_point = [theta_grid(i), v_grid(j), t1, t2, t3, tau1, tau2, tau3];
                                        start_points = [start_points; new_point];
                                        point_count = point_count + 1;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    
    % 如果当前迭代有最优解，将其加入起点
    if ~isempty(best_x)
        start_points = [best_x; start_points];
    end
    
    % 随机选择max_points个起点，避免过多
    if size(start_points, 1) > max_points
        rng(42 + iter); % 设置随机种子保证可重复性
        idx = randperm(size(start_points, 1), max_points);
        start_points = start_points(idx, :);
    end
    
    fprintf('生成 %d 个网格起点\n', size(start_points, 1));
    
    % --- 当前迭代的多起点优化 ---
    current_best_x = [];
    current_best_fval = inf;
    success_count = 0;
    
    fprintf('开始第 %d 次迭代优化...\n', iter);
    
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', 'notify', ...
        'MaxFunctionEvaluations', 1000, ...
        'MaxIterations', 200, ...
        'ConstraintTolerance', 1e-4, ...
        'StepTolerance', 1e-4);
    
    for i = 1:size(start_points, 1)
        fprintf('迭代%d - 优化起点 %d/%d...', iter, i, size(start_points, 1));
        
        try
            [x_opt, fval, exitflag] = fmincon(@(x) objective_function_3smoke(x), start_points(i,:), A, b, [], [], lb, ub, [], options);
            
            if exitflag > 0
                success_count = success_count + 1;
                if fval < current_best_fval
                    current_best_fval = fval;
                    current_best_x = x_opt;
                    fprintf(' 成功! 遮蔽时长 = %.2f 秒\n', -fval);
                else
                    fprintf(' 成功但非最优\n');
                end
            else
                fprintf(' 未收敛\n');
            end
            
        catch ME
            fprintf(' 失败: %s\n', ME.message);
        end
    end
    
    % 更新全局最优解
    if ~isempty(current_best_x) && current_best_fval < best_fval
        best_x = current_best_x;
        best_fval = current_best_fval;
        fprintf('第 %d 次迭代找到更优解: 遮蔽时长 = %.2f 秒\n', iter, -best_fval);
    else
        fprintf('第 %d 次迭代未找到更优解\n', iter);
    end
    
    fprintf('第 %d 次迭代完成: %d/%d 个起点成功优化\n', iter, success_count, size(start_points, 1));
end

% --- 输出最终结果 ---
fprintf('\n==================================================\n');
fprintf('*** 问题三最优解 (迭代细化后) ***\n');
fprintf('最大遮蔽时长: %.2f 秒\n', -best_fval);
fprintf('\n最优参数:\n');
fprintf('无人机航向角: %.2f rad (%.1f°)\n', best_x(1), best_x(1)*180/pi);
fprintf('无人机速度: %.2f m/s\n', best_x(2));
fprintf('烟幕弹1 - 投放时刻: %.2f s, 起爆延时: %.2f s\n', best_x(3), best_x(6));
fprintf('烟幕弹2 - 投放时刻: %.2f s, 起爆延时: %.2f s\n', best_x(4), best_x(7));
fprintf('烟幕弹3 - 投放时刻: %.2f s, 起爆延时: %.2f s\n', best_x(5), best_x(8));

%--- 验证最优解 ---
fprintf('\n验证最优解遮蔽效果...\n');
[total_time, time_intervals] = verify_solution_3smoke(best_x);
fprintf('验证遮蔽时长: %.2f 秒\n', total_time);

if ~isempty(time_intervals)
    fprintf('遮蔽时间段:\n');
    for i = 1:size(time_intervals, 1)
        fprintf('  [%.2f, %.2f] 秒\n', time_intervals(i,1), time_intervals(i,2));
    end
else
    fprintf('无连续遮蔽时间段\n');
end

% 保持原有的目标函数和验证函数不变
function F = objective_function_3smoke(x)
    % 解析决策变量
    theta = x(1);
    v_d = x(2);
    t_drop = x(3:5);
    tau = x(6:8);
    
    % 计算遮蔽时长
    shielding_duration = compute_shielding_duration_3smoke(theta, v_d, t_drop, tau);
    
    % 返回负值（fmincon最小化）
    F = -shielding_duration;
end

function duration = compute_shielding_duration_3smoke(theta, v_d, t_drop, tau)
    % 计算三烟幕弹的遮蔽时长 
    
    % 定义常量
    V_M = 300; V_sink = 3; R_eff = 10; Delta_T_eff = 20;
    P_O = [0, 0, 0]; CB = [0, 200, 0]; RT = 7; HT = 10;
    M1_P0 = [20000, 0, 2000]; FY1_P0 = [17800, 0, 1800];
    
    % 无人机速度向量
    V_UAV = [v_d * cos(theta), v_d * sin(theta), 0];
    
    % 计算三个烟幕的起爆时间和起爆点
    t_det = t_drop + tau;
    P_det = zeros(3, 3);
    
    for j = 1:3
        P_drop = FY1_P0 + V_UAV * t_drop(j);
        P_det(j, :) = get_SmokeDetPos(P_drop, t_drop(j), V_UAV, t_det(j));
    end
    
    % 确定时间扫描范围
    t_min = min(t_det);
    t_max = max(t_det) + Delta_T_eff;
    if t_min >= t_max
        duration = 0;
        return;
    end
    
    % 使用较小的时间步长提高精度
    dt = 0.1; % 时间步长
    t_eval = t_min:dt:t_max;
    
    % 初始化遮蔽标志数组
    is_shielded = zeros(size(t_eval));
    
    % 方法1: 直接使用多烟幕遮蔽判定（推荐）
    % 对每个时间点，使用 is_FullyShielded 判断是否被任一烟幕遮蔽
    for k = 1:length(t_eval)
        t = t_eval(k);
        
        % 导弹位置
        PM_t = get_MissilePos(M1_P0, t, V_M, P_O);
        
        % 构建当前时刻有效的烟幕中心列表
        C_list = [];
        for j = 1:3
            % 只考虑在有效时间窗内的烟幕
            if t >= t_det(j) && t <= t_det(j) + Delta_T_eff
                C_j = get_Smokecenter(P_det(j, :), t_det(j), t, V_sink);
                C_list = [C_list; C_j];
            end
        end
        
        % 如果有有效烟幕，调用 is_FullyShielded 进行判定
        if ~isempty(C_list)
            is_shielded(k) = is_FullyShielded(PM_t, C_list, R_eff, RT, HT, CB);
        else
            is_shielded(k) = 0;
        end
    end
    
    % 计算总遮蔽时长（已避免重复计算）
    total_duration = sum(is_shielded) * dt;
    
    % 返回总时长
    duration = total_duration;
end

function [total_time, time_intervals] = verify_solution_3smoke(x)
    % 验证解的精确实遮蔽时长（返回遮蔽时长和时间区间）
    
    % 定义常量
    V_M = 300; V_sink = 3; R_eff = 10; Delta_T_eff = 20;
    P_O = [0, 0, 0]; CB = [0, 200, 0]; RT = 7; HT = 10;
    M1_P0 = [20000, 0, 2000]; FY1_P0 = [17800, 0, 1800];
    
    theta = x(1);
    v_d = x(2);
    t_drop = x(3:5);
    tau = x(6:8);
    
    V_UAV = [v_d * cos(theta), v_d * sin(theta), 0];
    
    % 计算起爆点和时间
    t_det = t_drop + tau;
    P_det = zeros(3, 3);
    for j = 1:3
        P_drop = FY1_P0 + V_UAV * t_drop(j);
        P_det(j, :) = get_SmokeDetPos(P_drop, t_drop(j), V_UAV, t_det(j));
    end
    
    % 精确时间扫描（小步长）
    t_min = min(t_det);
    t_max = max(t_det) + Delta_T_eff;
    if t_min >= t_max
        total_time = 0;
        time_intervals = [];
        return;
    end
    
    dt = 0.1; % 更小的时间步长
    t_eval = t_min:dt:t_max;
    
    is_shielded = zeros(size(t_eval));
    
    for k = 1:length(t_eval)
        t = t_eval(k);
        PM_t = get_MissilePos(M1_P0, t, V_M, P_O);
        
        C_list = zeros(3, 3);
        valid_count = 0;
        for j = 1:3
            if t >= t_det(j) && t <= t_det(j) + Delta_T_eff
                C_list(j, :) = get_Smokecenter(P_det(j, :), t_det(j), t, V_sink);
                valid_count = valid_count + 1;
            else
                C_list(j, :) = [inf, inf, inf];
            end
        end
        
        if valid_count > 0 && is_FullyShielded(PM_t, C_list, R_eff, RT, HT, CB)
            is_shielded(k) = 1;
        end
    end
    
    % 计算连续遮蔽区间
    time_intervals = [];
    if any(is_shielded)
        start_idx = find(diff([0, is_shielded]) == 1);
        end_idx = find(diff([is_shielded, 0]) == -1);
        
        for i = 1:length(start_idx)
            start_time = t_eval(start_idx(i));
            end_time = t_eval(end_idx(i));
            time_intervals = [time_intervals; start_time, end_time];
        end
    end
    
    total_time = sum(is_shielded) * dt;
end