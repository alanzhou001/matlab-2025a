% =========================================================================
% 第五题：Stage 1 独立训练脚本 (增强版)
% 目标：训练 5 架无人机，使其各自具备基本的单机拦截能力
% =========================================================================
clear; clc; close all;

% --- 路径检查 ---
if exist('Optimization_Functions', 'dir')
    addpath('Optimization_Functions');
end
if exist('Models', 'dir')
    addpath('Models');
end

% --- 1. 初始化环境 ---
try
    env = Problem5_RL_Env(); % 确保文件名是 Problem5_RL_Env.m
catch ME
    error('环境初始化失败！请确保 Problem5_RL_Env.m 已正确保存且类名一致。\n错误信息: %s', ME.message);
end

ActionDim_UAV = env.UAV_ActionDim; % 8维

% 存储训练好的策略均值 (5架 x 8维)
Pretrained_Mu = zeros(5, ActionDim_UAV); 

fprintf('>>> [Stage 1] 启动单机独立训练...\n');
fprintf('    目标: 训练 5 架无人机，确保单机得分 > 0.5 秒\n');
fprintf('------------------------------------------------------------\n');

% --- 2. 遍历每一架无人机 ---
for u = 1:5
    retry_count = 0;
    max_retries = 5; % 最大重试次数
    success = false;
    
    while ~success && retry_count < max_retries
        if retry_count > 0
            fprintf('  [Retry] UAV-%d 训练失败，正在进行第 %d 次重试...\n', u, retry_count);
        else
            fprintf('  正在训练 UAV-%d (独立作战)...\n', u);
        end
        
        % === CEM 训练参数 ===
        Max_Gen = 80;           % 迭代次数
        Batch_Size = 500;       % 样本数
        Elite_Num = 65;         % 精英数
        Smoothing = 0.3;        % 平滑因子 (学习率)
        
        % 初始化分布 (归一化空间 [-1, 1])
        % 均值设为 0，标准差设为 2 (覆盖全域)
        mu = zeros(1, ActionDim_UAV);
        sigma = 2 * ones(1, ActionDim_UAV);
        
        % 如果是重试，增加初始随机性
        if retry_count > 0
            sigma = 3 * ones(1, ActionDim_UAV); 
        end
        
        best_score_this_run = -inf;
        
        % === 训练循环 ===
        for gen = 1:Max_Gen
            % 1. [关键修复] 手动采样 (Action = mu + Z * sigma)
            Z = randn(Batch_Size, ActionDim_UAV);
            
            % 显式扩展维度 (兼容旧版 MATLAB)
            Mu_Mat = repmat(mu, Batch_Size, 1);
            Sigma_Mat = repmat(sigma, Batch_Size, 1);
            
            actions = Mu_Mat + Z .* Sigma_Mat;
            
            % 2. 评估
            rewards = zeros(Batch_Size, 1);
            for k = 1:Batch_Size
                % 调用环境计算单机奖励
                rewards(k) = env.get_individual_reward(u, actions(k,:));
            end
            
            % 3. 筛选精英
            [sorted_r, idx] = sort(rewards, 'descend');
            elites = actions(idx(1:Elite_Num), :);
            
            max_r = sorted_r(1);
            
            % 4. 更新分布 (平滑更新)
            mu_new = mean(elites);
            sigma_new = std(elites);
            
            mu = Smoothing * mu_new + (1 - Smoothing) * mu;
            sigma = Smoothing * sigma_new + (1 - Smoothing) * sigma;
            
            % 记录本轮最佳
            if max_r > best_score_this_run
                best_score_this_run = max_r;
            end
            
            % (可选) 打印进度
            % if mod(gen, 10) == 0
            %     fprintf('    Gen %d: Max Reward = %.2f\n', gen, max_r);
            % end
        end
        
        % === 结果判定 ===
        % 阈值判定：至少遮蔽 0.5 秒才算学会
        if best_score_this_run > 0.5
            success = true;
            Pretrained_Mu(u, :) = mu; % 保存策略
            fprintf('  [Success] UAV-%d 训练完成. 最大遮蔽时长: %.2f 秒\n', u, best_score_this_run);
        else
            retry_count = retry_count + 1;
        end
    end
    
    % 如果重试多次仍失败
    if ~success
        warning('UAV-%d 经过 %d 次尝试仍无法学会拦截 (Score=0).', u, max_retries);
        fprintf('    -> 将分配随机策略，留待 Stage 2 修正.\n');
        Pretrained_Mu(u, :) = rand(1, ActionDim_UAV) * 2 - 1; 
    end
end

fprintf('------------------------------------------------------------\n');
fprintf('>>> [Stage 1] 全部完成.\n');

% --- 保存结果供 Stage 2 使用 ---
save('Stage1_Pretrained.mat', 'Pretrained_Mu');
fprintf('结果已保存至 Stage1_Pretrained.mat\n');

% --- 简单展示策略 ---
fprintf('\n预训练策略概览 (物理参数):\n');
for u = 1:5
    % 调用环境解码函数查看物理含义
    phys_params = env.denormalize_action(Pretrained_Mu(u,:));
    theta = phys_params(1);
    v = phys_params(2);
    fprintf('  UAV-%d: 航向 %5.1f°, 速度 %5.1f m/s\n', u, rad2deg(theta), v);
end


%% ========================================================================
%  STAGE 2: 协同训练
% =========================================================================
fprintf('\n>>> [Stage 2] 启动多机协同训练 (策略：允许重叠，严惩怠工)...\n');
ActionDim_Total = env.Total_ActionDim;
% 1. 组装初始策略
Joint_Mu_Init = reshape(Pretrained_Mu', 1, ActionDim_Total);
Joint_Sigma_Init = 0.8 * ones(1, ActionDim_Total);

% 2. 训练参数
Max_Gen_Joint = 5000;    
Batch_Size_Joint = 2000; % 样本量充足，保证搜索质量
Elite_Num_Joint = 300;

% --- [关键参数修改] ---
Penalty_Overlap = 0.1;  % 重叠惩罚很小 (允许配合带来的重叠)
Penalty_Lazy    = 10.0; % 怠工惩罚巨大 (每架不干活扣 10 分，迫使它们必须参与)

mu = Joint_Mu_Init;
sigma = Joint_Sigma_Init;

Best_Action_Global = []; % 历史最佳策略保留
Global_Best_Score = -inf;

History_Union = [];
History_Overlap = [];
History_Lazy = [];

for gen = 1:Max_Gen_Joint
    % 采样
    Z = randn(Batch_Size_Joint, ActionDim_Total);
    Mu_Exp = repmat(mu, Batch_Size_Joint, 1);
    Sigma_Exp = repmat(sigma, Batch_Size_Joint, 1);
    actions = Mu_Exp + Z .* Sigma_Exp;
    
    % 精英保留 (Elitism)
    if ~isempty(Best_Action_Global)
        actions(1, :) = Best_Action_Global; 
    end
    
    rewards = zeros(Batch_Size_Joint, 1);
    union_times = zeros(Batch_Size_Joint, 1);
    overlap_times = zeros(Batch_Size_Joint, 1);
    lazy_counts = zeros(Batch_Size_Joint, 1);
    
    % 评估
    for k = 1:Batch_Size_Joint
        % [调用新的奖励函数]
        [score, u_time, o_time, n_lazy] = env.get_cooperative_reward(actions(k,:), Penalty_Overlap, Penalty_Lazy);
        
        rewards(k) = score;       % 优化目标 (含惩罚)
        union_times(k) = u_time;  % 真实物理指标
        overlap_times(k) = o_time;
        lazy_counts(k) = n_lazy;
    end
    
    % 更新
    [sorted_r, idx] = sort(rewards, 'descend');
    elites = actions(idx(1:Elite_Num_Joint), :);
    
    % 记录最优 (基于真实的 Union Time 更新全局最优)
    current_best_union = union_times(idx(1));
    if current_best_union > Global_Best_Score
        Global_Best_Score = current_best_union;
        Best_Action_Phys = env.denormalize_action(actions(idx(1), :));
        Best_Action_Global = actions(idx(1), :);
    end
    
    mu_new = mean(elites);
    sigma_new = std(elites);
    
    alpha = 0.2;
    mu = alpha * mu_new + (1-alpha) * mu;
    sigma = alpha * sigma_new + (1-alpha) * sigma;
    
    % 衰减策略：后期可以稍微放宽 Lazy 惩罚，但保持 Overlap 惩罚较小
    if gen > Max_Gen_Joint * 0.8
        Penalty_Lazy = 5.0; % 后期稍微降低一点，防止过拟合
    end
    
    % 记录历史数据
    History_Union(gen) = current_best_union;
    History_Overlap(gen) = overlap_times(idx(1));
    History_Lazy(gen) = lazy_counts(idx(1));
    
    fprintf('Gen %d | Union: %.2fs | Overlap: %.2fs | Lazy Agents: %d | Score: %.2f\n', ...
        gen, current_best_union, History_Overlap(gen), History_Lazy(gen), sorted_r(1));
end

%% ========================================================================
%  结果展示
% =========================================================================
fprintf('\n================ 最终优化结果 ================\n');
fprintf('多机协同总有效遮蔽时长 (Union): %.4f 秒\n', Global_Best_Score);

% 绘图分析
figure;
subplot(2,1,1);
plot(History_Union, 'LineWidth', 2); hold on;
plot(History_Overlap, '--', 'LineWidth', 1.5);
ylabel('时间 (s)'); legend('有效遮蔽 (Union)', '重叠 (Overlap)');
title('协同效果分析'); grid on;

subplot(2,1,2);
plot(History_Lazy, 'r-', 'LineWidth', 1.5);
ylabel('怠工无人机数量'); xlabel('Generation');
title('全员参与度监控 (0表示全员干活)'); grid on;
ylim([-0.5 5.5]);

%% ========================================================================
%  结果展示 (修改部分)
% =========================================================================
fprintf('\n================ 最终优化结果 ================\n');
% 调用修改后的输出函数，传入 env 对象以便计算独立时长
Output_Strategy(env, Best_Action_Phys);

% 绘图
figure;
yyaxis left;
plot(History_Union, 'LineWidth', 2);
ylabel('有效遮蔽时长 (Union Time)');
yyaxis right;
plot(History_Overlap, '--', 'LineWidth', 1.5);
ylabel('重叠浪费时长 (Overlap Time)');
title('协同训练过程');
legend('Union Time', 'Overlap Time');
grid on;


% =========================================================================
% 辅助函数：输出详细策略 (包含独立遮蔽时长)
% =========================================================================
function Output_Strategy(env, x)
    fprintf('多机协同总有效遮蔽时长 (Union): %.4f 秒\n', env.calculate_shielding_time(compute_all_smokes(env, x)));
    fprintf('--------------------------------------------------------------------------------------\n');
    fprintf('%-6s | %-12s | %-10s | %-8s | %-25s\n', 'UAV ID', '独立遮蔽时长', '航向(deg)', '速度(m/s)', '投放时间序列(s)');
    fprintf('--------------------------------------------------------------------------------------\n');
    
    for i = 1:5
        idx = (i-1)*8 + 1;
        u_vars = x(idx : idx+7);
        
        % 1. 计算该 UAV 的烟幕
        Smokes_i = env.compute_smokes_for_uav(i, u_vars);
        
        % 2. 计算该 UAV 的独立遮蔽时长
        ind_time = env.calculate_shielding_time(Smokes_i);
        
        % 3. 准备参数用于显示
        theta_deg = rad2deg(u_vars(1));
        v = u_vars(2);
        t1 = u_vars(3);
        t2 = t1 + u_vars(4);
        t3 = t2 + u_vars(5);
        
        fprintf('UAV-%d  | %6.2f s     | %8.1f   | %6.1f   | [%.1f, %.1f, %.1f]\n', ...
            i, ind_time, theta_deg, v, t1, t2, t3);
    end
    fprintf('--------------------------------------------------------------------------------------\n');
end

% 辅助：计算所有烟幕用于总时长验证
function All_Smokes = compute_all_smokes(env, x)
    All_Smokes = [];
    for i = 1:5
        idx = (i-1)*8 + 1;
        Smokes_i = env.compute_smokes_for_uav(i, x(idx : idx+7));
        All_Smokes = [All_Smokes; Smokes_i];
    end
end