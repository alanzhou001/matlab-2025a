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
%  STAGE 2: GMM-CEM 多模态协同训练 (混合高斯升级版)
% =========================================================================
fprintf('\n>>> [Stage 2] 启动 GMM-CEM 协同训练 (混合高斯模型 K=3)...\n');

ActionDim_Total = env.Total_ActionDim;

% --- 1. GMM 参数初始化 ---
K = 3; % 定义 3 个高斯分量 (代表 3 种潜在战术流派)

% (1) 权重 Pi: 初始均分
GMM_Pi = ones(1, K) / K; 

% (2) 均值 Mu: K x 40
% 为了防止三个分量重合，我们在预训练结果的基础上添加不同的扰动
Pretrained_Vec = reshape(Pretrained_Mu', 1, ActionDim_Total);
GMM_Mu = repmat(Pretrained_Vec, K, 1);
GMM_Mu(2, :) = Pretrained_Vec + 0.5 * randn(1, ActionDim_Total); % 战术B：激进扰动
GMM_Mu(3, :) = Pretrained_Vec - 0.5 * randn(1, ActionDim_Total); % 战术C：反向扰动

% (3) 方差 Sigma: K x 40
GMM_Sigma = repmat(0.8 * ones(1, ActionDim_Total), K, 1);

% 训练参数
Max_Gen_Joint = 200;    % GMM 收敛较慢，但每代质量高
Batch_Size_Joint = 2000;
Elite_Num_Joint = 200;

% 惩罚系数
Penalty_Overlap = 0.1;
Penalty_Lazy    = 10.0;

Global_Best_Score = -inf;
Best_Action_Global = [];

history_union = [];

for gen = 1:Max_Gen_Joint
    
    % === Step 1: 混合采样 (Mixture Sampling) ===
    actions = zeros(Batch_Size_Joint, ActionDim_Total);
    
    % 根据权重 Pi，决定每个分量出多少样本 (多项分布采样)
    counts = mnrnd(Batch_Size_Joint, GMM_Pi); 
    
    curr_idx = 1;
    for k = 1:K
        n_k = counts(k);
        if n_k > 0
            % 从第 k 个高斯分量采样: mu_k + Z * sigma_k
            Z = randn(n_k, ActionDim_Total);
            mu_k = GMM_Mu(k, :);
            sig_k = GMM_Sigma(k, :);
            
            % 生成样本
            actions(curr_idx : curr_idx+n_k-1, :) = mu_k + Z .* sig_k;
            curr_idx = curr_idx + n_k;
        end
    end
    
    % 精英保留 (Elitism)
    if ~isempty(Best_Action_Global)
        actions(1, :) = Best_Action_Global;
    end
    
    % === Step 2: 评估 (Evaluation) ===
    rewards = zeros(Batch_Size_Joint, 1);
    unions = zeros(Batch_Size_Joint, 1);
    
    % 并行计算 (如果想加速可改为 parfor)
    for i = 1:Batch_Size_Joint
        [score, u_time, ~, ~] = env.get_cooperative_reward(actions(i,:), Penalty_Overlap, Penalty_Lazy);
        rewards(i) = score;
        unions(i) = u_time;
    end
    
    % === Step 3: 筛选精英 (Selection) ===
    [sorted_r, idx] = sort(rewards, 'descend');
    elites = actions(idx(1:Elite_Num_Joint), :);
    
    % 更新全局最优
    if unions(idx(1)) > Global_Best_Score
        Global_Best_Score = unions(idx(1));
        Best_Action_Global = actions(idx(1), :);
    end
    
    % === Step 4: EM 算法更新参数 (The Core) ===
    % 我们不知道哪个精英样本来自哪个分量，所以需要计算“责任度” (E-step)
    
    % 1. 计算对数似然 (Log-Likelihood) 防止数值下溢
    % Gamma(i, k) 表示第 i 个样本属于第 k 个分量的对数概率
    Log_Gamma = zeros(Elite_Num_Joint, K);
    
    for k = 1:K
        mu_k = GMM_Mu(k, :);
        sig_k = GMM_Sigma(k, :);
        
        % 高维高斯对数概率公式: -0.5 * sum( ((x-mu)/sig)^2 ) - sum(log(sig))
        diff = (elites - mu_k) ./ (sig_k + 1e-6); % 归一化距离
        log_prob = -0.5 * sum(diff.^2, 2) - sum(log(sig_k + 1e-6));
        
        % 加上先验权重的对数
        Log_Gamma(:, k) = log_prob + log(GMM_Pi(k) + 1e-6);
    end
    
    % 2. 归一化得到责任度 gamma_ik (利用 Log-Sum-Exp 技巧)
    % gamma_ik = P(k|x_i)
    Max_Log = max(Log_Gamma, [], 2);
    Gamma = exp(Log_Gamma - Max_Log); % 减去最大值防溢出
    Gamma = Gamma ./ sum(Gamma, 2);   % 按行归一化
    
    % 3. M-step: 加权更新参数
    % N_eff(k) 是第 k 个分量“抢到”的有效样本数
    N_eff = sum(Gamma, 1); 
    
    for k = 1:K
        % 防止某个分量死掉 (样本数过少)
        if N_eff(k) > 1.0 
            % 更新权重 Pi
            GMM_Pi(k) = N_eff(k) / Elite_Num_Joint;
            
            % 更新均值 Mu (加权平均)
            % mu_new = sum(gamma * x) / sum(gamma)
            weighted_sum_x = Gamma(:, k)' * elites;
            mu_new = weighted_sum_x / N_eff(k);
            
            % 更新方差 Sigma (加权方差)
            diff = elites - mu_new;
            weighted_sq_diff = Gamma(:, k)' * (diff.^2);
            sigma_new = sqrt(weighted_sq_diff / N_eff(k));
            
            % 平滑更新
            alpha = 0.3; % 学习率
            GMM_Mu(k, :) = alpha * mu_new + (1-alpha) * GMM_Mu(k, :);
            GMM_Sigma(k, :) = alpha * sigma_new + (1-alpha) * GMM_Sigma(k, :);
            
            % 增加一点底噪，防止方差收缩为0
            GMM_Sigma(k, :) = max(GMM_Sigma(k, :), 0.05);
        else
            % 如果分量死掉，随机重启它 (Re-initialization)
            GMM_Pi(k) = 0.05; % 给一个小权重
            GMM_Mu(k, :) = Best_Action_Global + randn(1, ActionDim_Total); % 在当前最优附近重生
            GMM_Sigma(k, :) = ones(1, ActionDim_Total);
        end
    end
    
    % 确保权重和为 1
    GMM_Pi = GMM_Pi / sum(GMM_Pi);
    
    % --- 打印进度 ---
    history_union(gen) = Global_Best_Score;
    fprintf('Gen %d | Best Union: %.2fs | Pi: [%.2f %.2f %.2f]\n', ...
        gen, Global_Best_Score, GMM_Pi);
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