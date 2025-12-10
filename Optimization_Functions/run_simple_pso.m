function [gbest, gbest_val, loss_history] = run_simple_pso(fun, nvars, lb, ub, options)
% RUN_SIMPLE_PSO 一个简易的不依赖工具箱的粒子群优化器
% 输入:
%   fun     - 目标函数句柄
%   nvars   - 变量个数
%   lb, ub  - 变量下界和上界 (向量)
%   options - 参数结构体 (SwarmSize, MaxIter, w, c1, c2)

    % 提取参数
    SwarmSize = options.SwarmSize;
    MaxIter = options.MaxIter;
    w = options.w;   % 惯性权重
    c1 = options.c1; % 自我认知参数
    c2 = options.c2; % 社会认知参数

    % 1. 初始化粒子群
    % 位置初始化: 在 [lb, ub] 之间随机生成
    pos = repmat(lb, SwarmSize, 1) + rand(SwarmSize, nvars) .* repmat(ub - lb, SwarmSize, 1);
    
    % 速度初始化: 初始速度设为0
    vel = zeros(SwarmSize, nvars);
    
    % 限制最大速度 (通常设为范围的 20%)
    vmax = 0.2 * (ub - lb);
    
    % 初始化个体最优 (pbest) 和全局最优 (gbest)
    pbest_pos = pos;
    pbest_val = inf(SwarmSize, 1);
    
    gbest_pos = zeros(1, nvars);
    gbest_val = inf;
    
    loss_history = zeros(MaxIter, 1);

    % --- 迭代循环 ---
    for iter = 1:MaxIter
        % 计算每个粒子的适应度
        for i = 1:SwarmSize
            % 边界强制约束 (Clipping)
            pos(i, :) = max(pos(i, :), lb);
            pos(i, :) = min(pos(i, :), ub);
            
            % 计算目标函数值
            val = fun(pos(i, :));
            
            % 更新个体最优
            if val < pbest_val(i)
                pbest_val(i) = val;
                pbest_pos(i, :) = pos(i, :);
            end
            
            % 更新全局最优
            if val < gbest_val
                gbest_val = val;
                gbest_pos = pos(i, :);
            end
        end
        
        loss_history(iter) = gbest_val;
        
        % 打印进度
        if mod(iter, 10) == 0 || iter == 1
            fprintf('迭代 %d/%d: 当前最优值 = %.4f\n', iter, MaxIter, -gbest_val);
        end
        
        % 更新速度和位置
        r1 = rand(SwarmSize, nvars);
        r2 = rand(SwarmSize, nvars);
        
        vel = w * vel + ...
              c1 * r1 .* (pbest_pos - pos) + ...
              c2 * r2 .* (repmat(gbest_pos, SwarmSize, 1) - pos);
          
        % 速度限制
        vel = max(vel, -repmat(vmax, SwarmSize, 1));
        vel = min(vel, repmat(vmax, SwarmSize, 1));
        
        pos = pos + vel;
    end
    
    gbest = gbest_pos;
end