function Population = Heuristic_Init_Randomized(PopSize, TotalVars, Params, lb, ub)
% 生成带有随机扰动的启发式种群
    
    Population = zeros(PopSize, TotalVars);
    Num_UAVs = 5;
    
    % 1. 基础随机填充 (70% 的粒子纯随机，保证探索性)
    Num_Random = ceil(PopSize * 0.7);
    for i = 1:Num_Random
        Population(i, :) = lb + rand(1, TotalVars) .* (ub - lb);
    end
    
    % 2. 启发式填充 (30% 的粒子基于几何猜测，但加入强噪声)
    Num_Heuristic = PopSize - Num_Random;
    
    % 任务分配池 (随机打乱分配逻辑)
    % 比如这次可能是 FY1->M3, FY2->M1 ...
    % 这里简单处理：让每架无人机随机选择一个导弹作为主要拦截目标
    
    for i = 1:Num_Heuristic
        x_candidate = [];
        
        for u = 1:Num_UAVs
            % 随机选择一个目标导弹进行瞄准
            m_idx = randi(3); 
            
            M_Pos = Params.Missiles(m_idx).Pos0;
            U_Pos = Params.UAVs(u).Pos0;
            
            % 随机选择拦截位置比例 (0.3 ~ 0.7 之间)
            ratio = 0.3 + 0.4 * rand; 
            Intercept_X = M_Pos(1) * (1-ratio) + Params.Target_Fake(1) * ratio;
            
            % 简单的拦截方向计算
            Vec_U = [Intercept_X, 0, 1800] - U_Pos; % 粗略目标点
            desired_theta = atan2(Vec_U(2), Vec_U(1));
            if desired_theta < 0, desired_theta = desired_theta + 2*pi; end
            
            % 加上随机噪声 (-10度 ~ +10度)
            noise_theta = (rand - 0.5) * 20 * (pi/180);
            theta = desired_theta + noise_theta;
            
            % 速度和时间也随机化
            v = 100 + 40 * rand; % 100-140 m/s
            t1 = 10 + 20 * rand; % 10-30s
            
            u_gene = [theta, v, t1, 2, 2, 3, 3, 3];
            x_candidate = [x_candidate, u_gene];
        end
        
        % 边界检查
        x_candidate = max(x_candidate, lb);
        x_candidate = min(x_candidate, ub);
        
        Population(Num_Random + i, :) = x_candidate;
    end
end