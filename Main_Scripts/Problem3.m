clear; clc;
rng(114);
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
num_iterations = 3;         % 迭代次数
K = 9;                      % 每次迭代保留的较优解数量
max_points = 2000;           % 最大随机起点数

% --- 变量边界约束 ---
lb = [179.5*pi/180, 70, 0, 2, 4, 3, 3, 3];
ub = [180.5*pi/180, 140, 2, 4, 6, 6, 6, 6];

% --- 线性不等式约束 (投放间隔 >= 1s) ---
A = [0, 0, 1, -1, 0, 0, 0, 0;     % t_drop2 - t_drop1 >= 1
     0, 0, 0, 1, -1, 0, 0, 0];    % t_drop3 - t_drop2 >= 1
b = [-1; -1];

% --- 初始化最优解列表 ---
best_solutions = [];   % 存储每次迭代的top K解
best_fvals = [];       % 对应的目标函数值

% --- 迭代细化搜索 ---
for iter = 1:num_iterations
    fprintf('\n=== 第 %d 次迭代细化搜索 ===\n', iter);
    
    % 确定当前搜索范围
    if isempty(best_solutions)
        % 第一次迭代使用全局范围
        current_lb = lb;
        current_ub = ub;
        search_centers = []; % 第一次迭代没有中心点
        search_intervals = []; % 第一次迭代没有区间集合
    else
        % 根据前一次迭代的top K解，生成搜索区间集合
        % 每个搜索区间是以一个最优解为中心，区间长度 = (ub-lb) * (0.5^(iter-1)) / K
        range_scale = (0.5 ^ (iter-1))/2;
        search_intervals = cell(size(best_solutions, 1), 1);
        
        for k = 1:size(best_solutions, 1)
            % 计算当前最优解对应的搜索区间
            interval_length = (ub - lb) * range_scale;
            sol_lb = max(lb, best_solutions(k, :) - interval_length/2);
            sol_ub = min(ub, best_solutions(k, :) + interval_length/2);
            
            % 确保投放时间间隔约束
            %sol_lb(4) = max(sol_lb(4), sol_lb(3) + 1);
            %sol_lb(5) = max(sol_lb(5), sol_lb(4) + 1);
            
            search_intervals{k} = struct('lb', sol_lb, 'ub', sol_ub);
            
            fprintf('中心点 %d 搜索区间:\n', k);
            fprintf('  航向角: [%.2f°, %.2f°]\n', sol_lb(1)*180/pi, sol_ub(1)*180/pi);
            fprintf('  速度: [%.1f, %.1f] m/s\n', sol_lb(2), sol_ub(2));
        end
        
        % 第一次迭代后的迭代，使用全局范围作为备选（避免某些维度搜索范围过小）
        current_lb = lb;
        current_ub = ub;
    end
    
    % --- 生成当前迭代的随机起点 ---
    fprintf('生成第 %d 次迭代的随机起点...\n', iter);
    
    start_points = [];
    
    % 如果当前迭代有最优解，将其加入起点
    if ~isempty(best_solutions)
        start_points = [best_solutions; start_points];
    end

    % 生成随机起点
    p = size(start_points, 1);
    
    if isempty(search_intervals)
        % 第一次迭代：在全局范围内生成随机起点
        while p < max_points
            % 对于每个变量，在范围内随机均匀采样
            theta_rand = current_lb(1) + rand() * (current_ub(1) - current_lb(1));
            v_rand = current_lb(2) + rand() * (current_ub(2) - current_lb(2));
            
            % 处理投放时间约束：t_drop2 >= t_drop1 + 1, t_drop3 >= t_drop2 + 1
            t1_rand = current_lb(3) + rand() * (current_ub(3) - current_lb(3));
            t2_rand = max(current_lb(4), t1_rand + 1) + rand() * (current_ub(4) - max(current_lb(4), t1_rand + 1));
            t3_rand = max(current_lb(5), t2_rand + 1) + rand() * (current_ub(5) - max(current_lb(5), t2_rand + 1));
            
            % 确保不超过上限
            t2_rand = min(t2_rand, current_ub(4));
            t3_rand = min(t3_rand, current_ub(5));
            
            % 起爆延时在范围内随机采样
            tau1_rand = current_lb(6) + rand() * (current_ub(6) - current_lb(6));
            tau2_rand = current_lb(7) + rand() * (current_ub(7) - current_lb(7));
            tau3_rand = current_lb(8) + rand() * (current_ub(8) - current_lb(8));

            if (t2_rand - t1_rand >= 1) && (t3_rand - t2_rand >= 1)
                p = p + 1; 
                new_point = [theta_rand, v_rand, t1_rand, t2_rand, t3_rand, tau1_rand, tau2_rand, tau3_rand];
                start_points = [start_points; new_point];
            end           
        end
    else
        % 后续迭代：在K个最优解对应的区间并集内生成随机起点
        while p < max_points
            % 随机选择一个最优解对应的区间
            interval_idx = randi(length(search_intervals));
            interval = search_intervals{interval_idx};
            
            % 在当前区间内随机采样
            theta_rand = interval.lb(1) + rand() * (interval.ub(1) - interval.lb(1));
            v_rand = interval.lb(2) + rand() * (interval.ub(2) - interval.lb(2));
            
            % 处理投放时间约束：t_drop2 >= t_drop1 + 1, t_drop3 >= t_drop2 + 1
            t1_rand = interval.lb(3) + rand() * (interval.ub(3) - interval.lb(3));
            t2_rand = max(interval.lb(4), t1_rand + 1) + rand() * (interval.ub(4) - max(interval.lb(4), t1_rand + 1));
            t3_rand = max(interval.lb(5), t2_rand + 1) + rand() * (interval.ub(5) - max(interval.lb(5), t2_rand + 1));
            
            % 确保不超过上限
            t2_rand = min(t2_rand, interval.ub(4));
            t3_rand = min(t3_rand, interval.ub(5));
            
            % 起爆延时在范围内随机采样
            tau1_rand = interval.lb(6) + rand() * (interval.ub(6) - interval.lb(6));
            tau2_rand = interval.lb(7) + rand() * (interval.ub(7) - interval.lb(7));
            tau3_rand = interval.lb(8) + rand() * (interval.ub(8) - interval.lb(8));

            if (t2_rand - t1_rand >= 1) && (t3_rand - t2_rand >= 1)
                p = p + 1; 
                new_point = [theta_rand, v_rand, t1_rand, t2_rand, t3_rand, tau1_rand, tau2_rand, tau3_rand];
                start_points = [start_points; new_point];
            end           
        end
    end
    
    fprintf('生成 %d 个随机起点\n', size(start_points, 1));
    
    % --- 当前迭代的多起点优化 ---
    current_solutions = [];
    current_fvals = [];
    success_count = 0;
    
    fprintf('开始第 %d 次迭代优化...\n', iter);
    
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'display', 'off', ...
        'MaxFunctionEvaluations', 1000, ...
        'MaxIterations', 200, ...
        'ConstraintTolerance', 1e-4, ...
        'StepTolerance', 1e-4);
    
    for i = 1:size(start_points, 1)
        %fprintf('迭代%d - 优化起点 %d/%d...', iter, i, size(start_points, 1));
        try
            [x_opt, fval, exitflag] = fmincon(@(x) objective_function_3smoke(x), start_points(i,:), A, b, [], [], lb, ub, [], options);
            
            if exitflag > 0
                success_count = success_count + 1;
                current_solutions = [current_solutions; x_opt];
                current_fvals = [current_fvals; fval];
            end
        catch ME
             %fprintf(' 失败: %s\n', ME.message);
        end
    end
    
    % 如果当前迭代有成功解，则选取top K个
    if ~isempty(current_fvals)
        [sorted_fvals, sorted_idx] = sort(current_fvals);
        topK = min(K, length(sorted_fvals));
        best_solutions = current_solutions(sorted_idx(1:topK), :);
        best_fvals = sorted_fvals(1:topK);
        
        fprintf('第 %d 次迭代完成: %d/%d 个起点成功优化\n', iter, success_count, size(start_points, 1));
        fprintf('最佳遮蔽时长: %.2f 秒\n', -best_fvals(1));
    else
        fprintf('第 %d 次迭代无成功解，终止迭代\n', iter);
        break;
    end
end

% --- 输出最终结果 ---
fprintf('\n==================================================\n');
fprintf('*** 问题三最优解 (迭代细化后) ***\n');
fprintf('最大遮蔽时长: %.2f 秒\n', -best_fvals(1));
fprintf('\n最优参数:\n');
fprintf('无人机航向角: %.2f rad (%.1f°)\n', best_solutions(1,1), best_solutions(1,1)*180/pi);
fprintf('无人机速度: %.2f m/s\n', best_solutions(1,2));
fprintf('烟幕弹1 - 投放时刻: %.2f s, 起爆延时: %.2f s\n', best_solutions(1,3), best_solutions(1,6));
fprintf('烟幕弹2 - 投放时刻: %.2f s, 起爆延时: %.2f s\n', best_solutions(1,4), best_solutions(1,7));
fprintf('烟幕弹3 - 投放时刻: %.2f s, 起爆延时: %.2f s\n', best_solutions(1,5), best_solutions(1,8));

%--- 验证最优解 ---
fprintf('\n验证最优解遮蔽效果...\n');
[total_time, time_intervals] = verify_solution_3smoke(best_solutions(1,:));
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
    
    dt = 0.01; % 更小的时间步长
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