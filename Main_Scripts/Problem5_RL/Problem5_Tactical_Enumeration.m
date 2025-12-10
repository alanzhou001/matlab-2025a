% =========================================================================
% 第五题：战术穷举与并行微调 (Tactical Enumeration & Parallel Finetuning)
% 核心思想：
% 1. 穷举所有可能的任务分配 (3^5 = 243种)
% 2. 几何解算每种分配的理想初值，筛选出 Top 5 潜力战术
% 3. 对 Top 5 战术分别进行 CEM 微调，确保找到全局最优
% =========================================================================
clear; clc; close all;
fprintf('\n>>> [Stage 2] 启动战术穷举与微调系统...\n');

% --- 1. 环境初始化 ---
if exist('Optimization_Functions', 'dir'), addpath('Optimization_Functions'); end
if exist('Models', 'dir'), addpath('Models'); end
try
    env = Problem5_RL_Env();
catch
    error('请确保当前目录下有修复版的 Problem5_RL_Env.m');
end
ActionDim = env.Total_ActionDim;

% --- 2. 第一步：战术穷举与筛选 ---
fprintf('>>> Step 1: 正在穷举所有 243 种战术组合...\n');

% 生成所有可能的分配向量 (5架飞机，每架可选目标 1,2,3)
% 结果是 243x5 的矩阵，每行代表一种分配，如 [1 1 2 3 2]
[A, B, C, D, E] = ndgrid(1:3, 1:3, 1:3, 1:3, 1:3);
All_Tactics = [A(:), B(:), C(:), D(:), E(:)];
Num_Tactics = size(All_Tactics, 1);

Tactic_Scores = zeros(Num_Tactics, 1);
Tactic_Actions = zeros(Num_Tactics, ActionDim);

% 快速评估每种战术的“几何理想解”
% 并行计算加速 (如无工具箱请改为 for)
parfor i = 1:Num_Tactics
    target_assign = All_Tactics(i, :);
    
    % 使用几何启发式生成该战术的理想参数
    % 这是一个确定性的计算，不需要随机性
    action_phys = Generate_Ideal_Phys(env, target_assign);
    action_norm = env.normalize_action_custom(action_phys); % 需辅助函数
    
    % 评估分数 (只看 Union Time，忽略惩罚，我们要找潜力股)
    [score, ~, ~] = env.get_cooperative_reward(action_norm, 0, 0);
    
    Tactic_Scores(i) = score;
    Tactic_Actions(i, :) = action_norm;
end

% 筛选 Top K 战术
Top_K = 5; 
[sorted_scores, idx] = sort(Tactic_Scores, 'descend');
Top_Tactics = All_Tactics(idx(1:Top_K), :);
Top_Actions = Tactic_Actions(idx(1:Top_K), :);

fprintf('    筛选完成。Top 5 战术潜力分：\n');
for k=1:Top_K
    fprintf('    战术 #%d: %s -> 预估 %.2f 秒\n', k, mat2str(Top_Tactics(k,:)), sorted_scores(k));
end

% --- 3. 第二步：多路并行微调 (Multi-Path Finetuning) ---
fprintf('\n>>> Step 2: 对 Top %d 战术进行独立微调...\n', Top_K);

Final_Best_Score = -inf;
Final_Best_Action = [];
Final_Best_Tactic = [];

% 对这 5 种战术分别跑 CEM
for k = 1:Top_K
    fprintf('\n--- 正在微调战术 %d / %d: %s ---\n', k, Top_K, mat2str(Top_Tactics(k,:)));
    
    % 初始化 CEM
    mu = Top_Actions(k, :);
    % 初始方差给小一点，因为几何解已经很准了，只需要微调
    sigma = 0.5 * ones(1, ActionDim); 
    
    Best_Local_Score = -inf;
    Best_Local_Action = [];
    
    % CEM 参数 (加大样本量，减少轮次，因为起点很好)
    Max_Gen = 30;         % 轮次不用多，30代足够收敛
    Batch_Size = 500;     % 样本量给足！防止走偏
    Elite_Num = 25;
    
    for gen = 1:Max_Gen
        % 采样 (90% 正态 + 10% 随机)
        Z = randn(Batch_Size, ActionDim);
        actions = repmat(mu, Batch_Size, 1) + Z .* repmat(sigma, Batch_Size, 1);
        
        % 评估
        rewards = zeros(Batch_Size, 1);
        for j = 1:Batch_Size
            % 此时只给一点点重叠惩罚，主要刷时长
            [r, ~, ~] = env.get_cooperative_reward(actions(j,:), 0.1, 5.0);
            rewards(j) = r;
        end
        
        % 更新
        [sorted_r, sort_idx] = sort(rewards, 'descend');
        elites = actions(sort_idx(1:Elite_Num), :);
        
        mu = 0.3 * mean(elites) + 0.7 * mu;
        sigma = 0.3 * std(elites) + 0.7 * sigma;
        
        % 记录本战术的最优
        % 注意：这里要取真实的 Union Time
        [~, real_union, ~] = env.get_cooperative_reward(actions(sort_idx(1),:), 0, 0);
        if real_union > Best_Local_Score
            Best_Local_Score = real_union;
            Best_Local_Action = env.denormalize_action(actions(sort_idx(1), :));
        end
        
        fprintf('    Gen %d: %.2f 秒 (Sigma: %.2f)\n', gen, real_union, mean(sigma));
    end
    
    % 更新全局最优
    if Best_Local_Score > Final_Best_Score
        Final_Best_Score = Best_Local_Score;
        Final_Best_Action = Best_Local_Action;
        Final_Best_Tactic = Top_Tactics(k, :);
    end
end

% --- 4. 结果输出 ---
fprintf('\n================ 最终胜利 ================\n');
fprintf('全局最优遮蔽时长: %.4f 秒\n', Final_Best_Score);
fprintf('最佳战术分配: %s (UAV1->M%d, UAV2->M%d...)\n', ...
    mat2str(Final_Best_Tactic), Final_Best_Tactic(1), Final_Best_Tactic(2));

Output_Strategy_Table(env, Final_Best_Action);


% =========================================================================
% 辅助函数 1: 几何理想解生成器 (确定性计算)
% =========================================================================
function x_phys = Generate_Ideal_Phys(env, task_assign)
    P = env.GlobalParams;
    x_phys = zeros(1, 40);
    
    for u = 1:5
        m_idx = task_assign(u);
        
        Pos_U = P.UAVs(u).Pos0;
        Pos_M = P.Missiles(m_idx).Pos0;
        Dist_M_Total = norm(Pos_M - P.Target_Fake);
        
        % 策略：根据 UAV 编号设定不同的拦截段，防止大家挤在一起
        % 如果多架飞机打同一个导弹，让它们在不同距离拦截
        same_target_uavs = find(task_assign == m_idx);
        rank_in_group = find(same_target_uavs == u);
        total_in_group = length(same_target_uavs);
        
        % 将拦截点均匀分布在 20% ~ 60% 的路程上
        if total_in_group == 1
            ratio = 0.4; % 只有一架，打中间
        else
            % 多架，均匀排开
            ratios = linspace(0.6, 0.2, total_in_group); 
            ratio = ratios(rank_in_group);
        end
        
        Pos_Intercept = Pos_M + (P.Target_Fake - Pos_M) * ratio;
        
        % 1. 航向
        Vec = Pos_Intercept - Pos_U;
        theta = atan2(Vec(2), Vec(1));
        
        % 2. 速度
        Dist_U = norm(Vec);
        Time_M = norm(Pos_Intercept - Pos_M) / P.V_M;
        v = Dist_U / Time_M;
        v = max(70, min(140, v)); % 约束
        
        % 反算实际到达时间
        t_fly = Dist_U / v;
        
        % 3. 投放时间 (提前 4s 投)
        t1 = max(0, t_fly - 4);
        
        % 4. 填入
        idx = (u-1)*8 + 1;
        x_phys(idx) = theta;
        x_phys(idx+1) = v;
        x_phys(idx+2) = t1;
        x_phys(idx+3) = 1.5; % 间隔
        x_phys(idx+4) = 1.5;
        x_phys(idx+5:idx+7) = 4; % 延时 4s
    end
end

% =========================================================================
% 辅助函数 2: 手动归一化 (配合 env)
% =========================================================================
function action_norm = normalize_action_custom(env, x_phys)
    % 物理 -> [-1, 1]
    range = env.ub - env.lb;
    action_norm = 2 * (x_phys - env.lb) ./ range - 1;
    action_norm = max(min(action_norm, 1), -1);
end

% =========================================================================
% 辅助函数 3: 输出表格 (保持不变)
% =========================================================================
function Output_Strategy_Table(env, x)
    fprintf('\n>>>>>>>>>>>>>> 最终投放策略表 <<<<<<<<<<<<<<\n');
    fprintf('| UAV | 航向(°) | 速度(m/s) | 弹1(s) | 弹2(s) | 弹3(s) | 独立遮蔽(s) |\n');
    fprintf('|-----|---------|-----------|--------|--------|--------|-------------|\n');
    for i = 1:5
        idx = (i-1)*8 + 1; u = x(idx : idx+7);
        t1 = u(3); t2 = t1+u(4); t3 = t2+u(5);
        smokes = env.compute_smokes_for_uav(i, u);
        ind = env.calculate_shielding_time(smokes);
        fprintf('|  %d  | %7.1f | %9.1f | %6.1f | %6.1f | %6.1f | %11.2f |\n', ...
            i, rad2deg(u(1)), u(2), t1, t2, t3, ind);
    end
end