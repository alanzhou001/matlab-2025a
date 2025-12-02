% =========================================================================
% 第四题：无人机烟幕干扰导弹优化设计
% 功能：通过遗传算法优化3架无人机的烟幕投放参数，最大化对真目标的有效遮蔽时间
% 逻辑：导弹沿固定方向飞行，无人机投放烟幕形成遮蔽，需满足≥95%目标采样点被烟幕遮挡
% 变量：每架无人机的飞行方向角、飞行速度、烟幕投放时间、起爆延迟时间（共12个变量）
% =========================================================================
clear all; close all; clc;
rng(10000, 'twister');  % 设置随机种子，确保优化结果可复现

% ===================== 1. 物理与几何参数定义 =====================
% 基础物理常量
g = 9.8;                  % 重力加速度 (m/s²)
v_M1 = 300;               % 导弹飞行速度 (m/s)
R_cloud = 10;             % 烟幕有效遮蔽半径 (m)
v_sink = 3;               % 烟幕下沉速度 (m/s)
cloud_lifetime = 20;      % 烟幕有效持续时间 (s)

% 目标参数（真目标为圆柱形）
cylinder_radius = 7;      % 目标圆柱底面半径 (m)
cylinder_height = 10;     % 目标圆柱高度 (m)
T_base = [0, 200, 0];     % 真目标圆柱底部中心坐标 (x,y,z) (m)
O = [0, 0, 0];            % 假目标原点坐标 (m)

% 导弹与无人机初始位置
r_M1_0 = [20000, 0, 2000];% 导弹初始位置 (x,y,z) (m)
r_FY1_0 = [17800, 0, 1800];% 无人机1初始位置 (m)
r_FY2_0 = [12000, 1400, 1400];% 无人机2初始位置 (m)
r_FY3_0 = [6000, -3000, 700];% 无人机3初始位置 (m)

% 导弹飞行方向单位向量（指向假目标O）
e_M1 = (O - r_M1_0) / norm(O - r_M1_0);

% ===================== 2. 时间参数配置 =====================
t_start = 0;              % 计算起始时间 (s)
t_end = 100;              % 计算终止时间 (s)
dt = 0.01;                % 时间步长 (s)（高精度确保遮蔽时间计算准确）
t_range = t_start:dt:t_end;% 时间序列
n_t = length(t_range);    % 时间序列长度

% 估计导弹飞行至真目标的时间
dist_to_target = norm(r_M1_0 - T_base);  % 导弹初始位置到真目标的直线距离
est_flight_time = dist_to_target / v_M1; % 飞行时间估计（匀速飞行假设）
disp(['估计导弹飞行时间: ', num2str(est_flight_time), ' 秒']);

% ===================== 3. 遗传算法优化参数配置 =====================
% 决策变量上下界（12个变量：3架无人机×4个参数（方向角、速度、投放时间、延迟时间））
lb = [0.05, 80, 0.5, 0.3, ...  % 无人机1：方向角(rad)、速度(m/s)、投放时间(s)、延迟时间(s)
      4, 70, 10, 5, ...        % 无人机2：同上
      1.6, 80, 20, 5];         % 无人机3：同上
ub = [0.15, 120, 1, 0.6, ...   % 无人机1变量上界
      4.5, 90, 15, 9, ...      % 无人机2变量上界
      2.2, 120, 30, 8];        % 无人机3变量上界

% 遗传算法选项设置
options = optimoptions('ga', ...
    'MaxGenerations', 50, ...          % 最大迭代代数（控制优化迭代次数）
    'PopulationSize', 200, ...         % 种群大小（每次迭代的候选解数量）
    'Display', 'iter', ...             % 迭代过程显示（输出每代最优适应度）
    'PlotFcn', @gaplotbestf, ...       % 绘制最优适应度变化曲线
    'UseParallel', false, ...          % 禁用并行计算（避免环境配置问题）
    'MutationFcn', @mutationgaussian, ... % 变异函数（高斯变异）
    'CrossoverFcn', @crossoverscattered, ... % 交叉函数（离散交叉）
    'SelectionFcn', @selectionstochunif);   % 选择函数（随机均匀选择）

% ===================== 4. 目标函数定义=====================
% 遗传算法默认最小化，因此返回负的有效遮蔽时间（等价于最大化遮蔽时间）
fitness_function = @(theta) -calculate_multiple_UAVs_shielding_time_new(theta, ...
    r_M1_0, e_M1, v_M1, r_FY1_0, r_FY2_0, r_FY3_0, g, R_cloud, v_sink, cloud_lifetime, ...
    T_base, cylinder_radius, cylinder_height, t_range, dt);

% ===================== 5. 执行遗传算法优化=====================
fprintf('开始遗传算法优化...\n');
fprintf('种群大小: %d, 最大代数: %d\n', 200, 50);
tic;  % 计时开始（统计优化总耗时）
% 调用ga函数：输入目标函数、变量数、约束条件、选项
[optimal_params, fval] = ga(fitness_function, 12, [], [], [], [], lb, ub, [], options);
optimization_time = toc;  % 计时结束

% ===================== 6. 优化结果解析与输出=====================
% 解包最优参数（按无人机分类）
% 无人机1最优参数
angle1 = optimal_params(1);       % 飞行方向角 (rad)
angle1_deg = angle1 * 180 / pi;   % 转换为角度 (deg)
speed1 = optimal_params(2);       % 飞行速度 (m/s)
release_time1 = optimal_params(3);% 烟幕投放时间 (s)
delay_time1 = optimal_params(4);  % 投放到起爆的延迟时间 (s)
explosion_time1 = release_time1 + delay_time1; % 烟幕起爆时间 (s)

% 无人机2最优参数
angle2 = optimal_params(5);
angle2_deg = angle2 * 180 / pi;
speed2 = optimal_params(6);
release_time2 = optimal_params(7);
delay_time2 = optimal_params(8);
explosion_time2 = release_time2 + delay_time2;

% 无人机3最优参数
angle3 = optimal_params(9);
angle3_deg = angle3 * 180 / pi;
speed3 = optimal_params(10);
release_time3 = optimal_params(11);
delay_time3 = optimal_params(12);
explosion_time3 = release_time3 + delay_time3;

% 输出优化结果
fprintf('\n优化结果分析:\n');
fprintf('============================================\n');
fprintf('总有效遮蔽时间: %.4f 秒\n', -fval);  % fval为负的遮蔽时间，取反还原
fprintf('优化用时: %.2f 分钟\n', optimization_time/60);

fprintf('\n  无人机FY1最优参数:\n');
fprintf('  飞行方向角: %.2f 度 (%.4f 弧度)\n', angle1_deg, angle1);
fprintf('  飞行速度: %.2f m/s\n', speed1);
fprintf('  烟幕干扰弹投放时间: %.3f s\n', release_time1);
fprintf('  投放到起爆的延迟时间: %.3f s\n', delay_time1);
fprintf('  烟幕干扰弹起爆时间: %.3f s\n', explosion_time1);

fprintf('\n  无人机FY2最优参数:\n');
fprintf('  飞行方向角: %.2f 度 (%.4f 弧度)\n', angle2_deg, angle2);
fprintf('  飞行速度: %.2f m/s\n', speed2);
fprintf('  烟幕干扰弹投放时间: %.3f s\n', release_time2);
fprintf('  投放到起爆的延迟时间: %.3f s\n', delay_time2);
fprintf('  烟幕干扰弹起爆时间: %.3f s\n', explosion_time2);

fprintf('\n  无人机FY3最优参数:\n');
fprintf('  飞行方向角: %.2f 度 (%.4f 弧度)\n', angle3_deg, angle3);
fprintf('  飞行速度: %.2f m/s\n', speed3);
fprintf('  烟幕干扰弹投放时间: %.3f s\n', release_time3);
fprintf('  投放到起爆的延迟时间: %.3f s\n', delay_time3);
fprintf('  烟幕干扰弹起爆时间: %.3f s\n', explosion_time3);

% 计算烟幕投放点与起爆点坐标（物理运动模型）
e_FY1 = [cos(angle1), sin(angle1), 0];  % 无人机1飞行方向单位向量（水平方向）
release_pos1 = r_FY1_0 + speed1 * release_time1 * e_FY1;  % 投放点坐标（匀速直线运动）
% 起爆点坐标：投放后继续飞行delay_time1秒，叠加重力下落（竖直方向加速度-g）
explosion_pos1 = release_pos1 + delay_time1 * speed1 * e_FY1 - 0.5 * g * delay_time1^2 * [0, 0, 1];

e_FY2 = [cos(angle2), sin(angle2), 0];
release_pos2 = r_FY2_0 + speed2 * release_time2 * e_FY2;
explosion_pos2 = release_pos2 + delay_time2 * speed2 * e_FY2 - 0.5 * g * delay_time2^2 * [0, 0, 1];

e_FY3 = [cos(angle3), sin(angle3), 0];
release_pos3 = r_FY3_0 + speed3 * release_time3 * e_FY3;
explosion_pos3 = release_pos3 + delay_time3 * speed3 * e_FY3 - 0.5 * g * delay_time3^2 * [0, 0, 1];

% 输出投放点与起爆点坐标
fprintf('\n 无人机FY1 烟幕干扰弹:\n');
fprintf('  投放点坐标: (%.2f, %.2f, %.2f) m\n', release_pos1);
fprintf('  起爆点坐标: (%.2f, %.2f, %.2f) m\n', explosion_pos1);

fprintf('\n 无人机FY2 烟幕干扰弹:\n');
fprintf('  投放点坐标: (%.2f, %.2f, %.2f) m\n', release_pos2);
fprintf('  起爆点坐标: (%.2f, %.2f, %.2f) m\n', explosion_pos2);

fprintf('\n 无人机FY3 烟幕干扰弹:\n');
fprintf('  投放点坐标: (%.2f, %.2f, %.2f) m\n', release_pos3);
fprintf('  起爆点坐标: (%.2f, %.2f, %.2f) m\n', explosion_pos3);

% 保存优化结果（便于后续分析与复现）
save('problem4_refactored_results.mat', 'optimal_params', 'fval', 'optimization_time');
fprintf('\n 结果已保存至 problem4_refactored_results.mat\n');

% =========================================================================
% 子函数1：计算多无人机烟幕的总有效遮蔽时间
% 输入：
%   theta - 12维决策变量（3架无人机×4个参数）
%   其余参数 - 物理/几何参数、时间序列（与主函数一致）
% 输出：
%   effective_time - 总有效遮蔽时间（s）（≥95%目标采样点被遮蔽的累计时间）
% =========================================================================
function effective_time = calculate_multiple_UAVs_shielding_time_new(theta, ...
    r_M1_0, e_M1, v_M1, r_FY1_0, r_FY2_0, r_FY3_0, g, R_cloud, v_sink, cloud_lifetime, ...
    T_base, cylinder_radius, cylinder_height, t_range, dt)
    
    % 1. 解包当前决策变量（候选解）
    angle1 = theta(1); speed1 = theta(2); t_release1 = theta(3); t_delay1 = theta(4);
    angle2 = theta(5); speed2 = theta(6); t_release2 = theta(7); t_delay2 = theta(8);
    angle3 = theta(9); speed3 = theta(10); t_release3 = theta(11); t_delay3 = theta(12);
    
    % 计算每架无人机的烟幕起爆时间（投放时间+延迟时间）
    t_explosion1 = t_release1 + t_delay1;
    t_explosion2 = t_release2 + t_delay2;
    t_explosion3 = t_release3 + t_delay3;
    
    % 2. 计算无人机飞行方向与烟幕起爆点坐标
    e_FY1 = [cos(angle1), sin(angle1), 0];  % 无人机1水平飞行方向单位向量
    r_FY1_release = r_FY1_0 + speed1 * t_release1 * e_FY1;  % 烟幕投放点坐标
    % 烟幕起爆点：投放后无人机继续飞行，烟幕受重力下落
    r_S1_explosion = r_FY1_release + t_delay1 * speed1 * e_FY1 - 0.5 * g * t_delay1^2 * [0, 0, 1];
    
    % 无人机2同理
    e_FY2 = [cos(angle2), sin(angle2), 0];
    r_FY2_release = r_FY2_0 + speed2 * t_release2 * e_FY2;
    r_S2_explosion = r_FY2_release + t_delay2 * speed2 * e_FY2 - 0.5 * g * t_delay2^2 * [0, 0, 1];
    
    % 无人机3同理
    e_FY3 = [cos(angle3), sin(angle3), 0];
    r_FY3_release = r_FY3_0 + speed3 * t_release3 * e_FY3;
    r_S3_explosion = r_FY3_release + t_delay3 * speed3 * e_FY3 - 0.5 * g * t_delay3^2 * [0, 0, 1];
    
    % 3. 生成目标圆柱的采样点（用于判断遮蔽效果）
    key_points = generateCylinderSamplingPoints(T_base, cylinder_radius, cylinder_height);
    
    % 4. 初始化有效遮蔽时间标记（每一时间步是否有效）
    n_t = length(t_range);
    is_effective = zeros(n_t, 1);  % 0=无效，1=有效
    
    % 5. 遍历每一时间步，判断是否有效遮蔽
    for i = 1:n_t
        t = t_range(i);  % 当前时间
        
        % 5.1 计算当前时刻导弹位置（匀速直线飞行）
        r_M1 = r_M1_0 + v_M1 * t * e_M1;
        
        % 5.2 收集当前时刻有效的烟幕中心（仅在烟幕有效期内）
        smoke_centers = [];
        % 无人机1烟幕：起爆后≤20秒内有效，且随时间下沉
        if t >= t_explosion1 && t <= t_explosion1 + cloud_lifetime
            r_C1 = r_S1_explosion - v_sink * (t - t_explosion1) * [0, 0, 1];
            smoke_centers = [smoke_centers; r_C1];
        end
        % 无人机2烟幕同理
        if t >= t_explosion2 && t <= t_explosion2 + cloud_lifetime
            r_C2 = r_S2_explosion - v_sink * (t - t_explosion2) * [0, 0, 1];
            smoke_centers = [smoke_centers; r_C2];
        end
        % 无人机3烟幕同理
        if t >= t_explosion3 && t <= t_explosion3 + cloud_lifetime
            r_C3 = r_S3_explosion - v_sink * (t - t_explosion3) * [0, 0, 1];
            smoke_centers = [smoke_centers; r_C3];
        end
        
        % 5.3 判断当前时刻是否有效遮蔽（≥95%采样点被遮蔽）
        if ~isempty(smoke_centers)  % 存在有效烟幕时才判断
            is_effective(i) = isObscuredByMultipleSmokes(r_M1, smoke_centers, key_points, R_cloud);
        end
    end
    
    % 6. 计算总有效遮蔽时间（有效时间步×时间步长）
    effective_time = sum(is_effective) * dt;
end

% =========================================================================
% 子函数2：生成目标圆柱的采样点（覆盖圆柱表面及轴线，共29个点）
% 输入：
%   target_center - 圆柱底部中心坐标 (x,y,z)
%   target_radius - 圆柱底面半径 (m)
%   cylinder_height - 圆柱高度 (m)
% 输出：
%   key_points - 29×3矩阵，每行是一个采样点坐标
% =========================================================================
function key_points = generateCylinderSamplingPoints(target_center, target_radius, cylinder_height)
    key_points = [];
    n_samples = 8;  % 每个圆周的采样点数量（保证覆盖均匀）
    
    % 1. 圆柱底部圆周采样（z=底部高度）
    for i = 1:n_samples
        angle = 2 * pi * (i-1) / n_samples;  % 均匀分布的角度
        x = target_center(1) + target_radius * cos(angle);
        y = target_center(2) + target_radius * sin(angle);
        z = target_center(3);
        key_points = [key_points; x, y, z];
    end
    
    % 2. 圆柱顶部圆周采样（z=底部高度+圆柱高度）
    for i = 1:n_samples
        angle = 2 * pi * (i-1) / n_samples;
        x = target_center(1) + target_radius * cos(angle);
        y = target_center(2) + target_radius * sin(angle);
        z = target_center(3) + cylinder_height;
        key_points = [key_points; x, y, z];
    end
    
    % 3. 圆柱中部圆周采样（z=底部高度+圆柱高度/2）
    for i = 1:n_samples
        angle = 2 * pi * (i-1) / n_samples;
        x = target_center(1) + target_radius * cos(angle);
        y = target_center(2) + target_radius * sin(angle);
        z = target_center(3) + cylinder_height / 2;
        key_points = [key_points; x, y, z];
    end
    
    % 4. 圆柱轴线采样（5个点，覆盖全高度）
    h_ratios = [0, 0.25, 0.5, 0.75, 1.0];  % 高度比例（0=底部，1=顶部）
    for i = 1:length(h_ratios)
        z = target_center(3) + cylinder_height * h_ratios(i);
        key_points = [key_points; target_center(1), target_center(2), z];
    end
end

% =========================================================================
% 子函数3：判断多烟幕对目标的遮蔽效果（是否≥95%采样点被遮蔽）
% 输入：
%   missile_position - 当前导弹位置 (x,y,z)
%   smoke_centers - 有效烟幕中心坐标矩阵（n×3，n为有效烟幕数量）
%   key_points - 目标采样点矩阵（29×3）
%   smoke_radius - 烟幕半径 (m)
% 输出：
%   obscured - 逻辑值（true=有效遮蔽，false=无效）
% =========================================================================
function obscured = isObscuredByMultipleSmokes(missile_position, smoke_centers, key_points, smoke_radius)
    blocked_count = 0;  % 被遮蔽的采样点数量
    total_rays = size(key_points, 1);  % 总采样点数量（29）
    
    % 遍历每个采样点，判断是否被任一烟幕遮蔽
    for i = 1:total_rays
        target_point = key_points(i, :);  % 当前采样点
        ray_blocked = false;  % 标记该采样点是否被遮蔽
        
        % 遍历每个有效烟幕，判断导弹到采样点的射线是否与烟幕相交
        for j = 1:size(smoke_centers, 1)
            smoke_center = smoke_centers(j, :);  % 当前烟幕中心
            % 调用射线-球体相交判断函数
            if isRayBlockedBySphere(missile_position, target_point, smoke_center, smoke_radius)
                ray_blocked = true;  % 被当前烟幕遮蔽
                break;  % 无需判断其他烟幕，跳出循环
            end
        end
        
        % 统计被遮蔽的采样点数量
        if ray_blocked
            blocked_count = blocked_count + 1;
        end
    end
    
    % 判断遮蔽比例是否≥95%
    obscuration_threshold = 0.95;
    obscured = (blocked_count / total_rays) >= obscuration_threshold;
end

% =========================================================================
% 子函数4：判断射线（导弹→采样点）是否与球体（烟幕）相交
% 数学模型：射线与球体相交的几何判定（空间解析几何）
% 输入：
%   ray_start - 射线起点（导弹位置）(x,y,z)
%   ray_end - 射线终点（采样点位置）(x,y,z)
%   sphere_center - 球心（烟幕中心）(x,y,z)
%   sphere_radius - 球半径（烟幕半径）(m)
% 输出：
%   blocked - 逻辑值（true=相交，false=不相交）
% =========================================================================
function blocked = isRayBlockedBySphere(ray_start, ray_end, sphere_center, sphere_radius)
    ray_vec = ray_end - ray_start;  % 射线向量（导弹→采样点）
    ray_length = norm(ray_vec);    % 射线长度（导弹到采样点的距离）
    
    % 特殊情况：射线起点与终点重合（采样点=导弹位置，实际不会发生）
    if ray_length == 0
        blocked = norm(sphere_center - ray_start) <= sphere_radius;
        return;
    end
    
    % 射线单位化（方向不变，长度=1）
    ray_dir = ray_vec / ray_length;
    % 球心到射线起点的向量
    to_sphere = sphere_center - ray_start;
    % 球心在射线上的投影长度（投影点到起点的距离）
    projection_length = dot(to_sphere, ray_dir);
    % 球心在射线上的投影点坐标
    closest_point_on_ray = ray_start + projection_length * ray_dir;
    % 球心到射线的最短距离（投影点到球心的距离）
    distance_to_ray = norm(sphere_center - closest_point_on_ray);
    
    % 最短距离>烟幕半径：无交点
    if distance_to_ray > sphere_radius
        blocked = false;
        return;
    end
    
    % 计算射线与球体的交点投影长度（相对于射线起点）
    half_chord_length = sqrt(sphere_radius^2 - distance_to_ray^2);  % 弦长的一半
    intersection1_proj = projection_length - half_chord_length;  % 第一个交点的投影长度
    intersection2_proj = projection_length + half_chord_length;  % 第二个交点的投影长度
    
    % 判断交点是否在射线范围内（投影长度≥0且≤射线长度）
    blocked = (0 <= intersection1_proj && intersection1_proj <= ray_length) || ...
              (0 <= intersection2_proj && intersection2_proj <= ray_length);
end