function F_negM = fun_NegativeM(PT, PM, C_list, Alpha_list)
% FUN_NEGATIVEM 遮蔽裕度目标函数：计算 -M_joint(PT)，用于 fmincon 最小化。

    L = PT - PM;
    norm_L = norm(L);
    
    Num_Smokes = size(C_list, 1);
    M_list = zeros(Num_Smokes, 1);
    
    if Num_Smokes == 0
        F_negM = -100; % 确保 max M 为大正数，判定未遮蔽
        return;
    end
    
    for j = 1:Num_Smokes
        Cj = C_list(j, :);
        cos_alpha_j = Alpha_list(j);
        
        A = Cj - PM;
        norm_A = norm(A);
        
        % 计算 cos(theta_j)
        if norm_L * norm_A < eps 
            cos_theta_j = 1; 
        else
            cos_theta_j = dot(L, A) / (norm_L * norm_A);
        end
        
        M_list(j) = cos_theta_j - cos_alpha_j;
    end
    
    M_joint = min(M_list);
    F_negM = -M_joint; % fmincon 最小化 -M_joint
end