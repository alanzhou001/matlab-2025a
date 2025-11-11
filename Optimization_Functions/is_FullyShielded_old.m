function is_shielded = is_FullyShielded(PM, C_list, R_eff, RT, HT, CB)
% IS_FULLYSHIELDED 判定是否完全遮蔽 (多点启动的精确优化判定)。

    Num_Smokes = size(C_list, 1);
    if Num_Smokes == 0
        is_shielded = false;
        return;
    end
    
    % 1. 计算所有有效烟幕的 cos(alpha)
    Alpha_list = zeros(Num_Smokes, 1);
    for j = 1:Num_Smokes
        D_MC = norm(C_list(j, :) - PM);
        if D_MC <= R_eff
            is_shielded = true; % 导弹在烟幕内
            return;
        end
        sin_alpha = R_eff / D_MC;
        Alpha_list(j) = sqrt(1 - sin_alpha^2); % cos(alpha)
    end
    
    % 2. 定义优化问题
    initial_point = CB + [0, 0, HT/2]; 
    
    % 线性约束 (高度)
    A_ineq = [0, 0, -1; 0, 0, 1];
    b_ineq = [0; HT];
    
    problem = createOptimProblem('fmincon', ...
        'objective', @(PT) fun_NegativeM(PT, PM, C_list, Alpha_list), ...
        'x0', initial_point, ...
        'Aineq', A_ineq, 'bineq', b_ineq, ...
        'nonlcon', @(PT) cylinder_constraint(PT, RT, CB, HT));
    
    % 3. 配置 MultiStart 并运行
    
    Num_start = 20; 
    StartPoints_Matrix = zeros(Num_start + 1, 3);
    for i = 1:Num_start
        theta = 2 * pi * (i / Num_start);
        StartPoints_Matrix(i, :) = CB + [RT*cos(theta), RT*sin(theta), HT];
    end
    StartPoints_Matrix(Num_start+1, :) = initial_point; % 目标中心
    
    % *** 修正点：简化 CustomStartPointSet 调用，只传入矩阵 ***
    CustomStartSet = CustomStartPointSet(StartPoints_Matrix); 
    
    ms = MultiStart('Display', 'off');
    
    % 使用不同的优化选项可能提高速度或鲁棒性
    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');
    ms.TolFun = 1e-8; % 提高收敛精度
    
    [~, fval_min_global] = run(ms, problem, CustomStartSet); 
    
    % 4. 最终判定
    M_max = -fval_min_global;
    
    if M_max <= 1e-8 
        is_shielded = true;
    else
        is_shielded = false;
    end
end