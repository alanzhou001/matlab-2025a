function [gbest, gbest_val, loss_history] = run_simple_pso(fun, nvars, lb, ub, options)

    % 提取参数
    SwarmSize = options.SwarmSize;
    MaxIter = options.MaxIter;
    c1 = options.c1; 
    c2 = options.c2; 

    w_max = 0.9; % 初始惯性大（全局搜索）
    w_min = 0.4; % 结束惯性小（局部收敛）
    
    % 初始化
    pos = repmat(lb, SwarmSize, 1) + rand(SwarmSize, nvars) .* repmat(ub - lb, SwarmSize, 1);
    vel = zeros(SwarmSize, nvars);
    vmax = 0.2 * (ub - lb); % 速度钳制
    
    pbest_pos = pos;
    pbest_val = inf(SwarmSize, 1);
    gbest_pos = zeros(1, nvars);
    gbest_val = inf;
    
    loss_history = zeros(MaxIter, 1);

    % --- 迭代循环 ---
    for iter = 1:MaxIter
        % 动态计算当前权重 w 
        w = w_max - (w_max - w_min) * (iter / MaxIter);
        
        for i = 1:SwarmSize
            % 边界处理
            pos(i, :) = max(pos(i, :), lb);
            pos(i, :) = min(pos(i, :), ub);
            
            % 计算适应度
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
        
        % 速度更新 
        r1 = rand(SwarmSize, nvars);
        r2 = rand(SwarmSize, nvars);
        vel = w * vel + c1 * r1 .* (pbest_pos - pos) + c2 * r2 .* (repmat(gbest_pos, SwarmSize, 1) - pos);
        
        % 速度限制
        vel = max(vel, -repmat(vmax, SwarmSize, 1));
        vel = min(vel, repmat(vmax, SwarmSize, 1));
        
        pos = pos + vel;
    end
    gbest = gbest_pos;
end