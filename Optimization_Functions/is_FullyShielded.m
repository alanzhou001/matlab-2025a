function is_shielded = is_FullyShielded_MultiSmoke(PM, C_list, R_eff, RT, HT, CB)
% IS_FULLYSHIELDED_MULTISMOKE 判定是否完全遮蔽 (多烟幕支持的采样比例模型)。
% 适用于多弹 C_list (Nx3) 和单导弹 PM 的遮蔽判定。

    % --- 采样参数设置 ---
    N_THETA_DIVISIONS = 30;     % 环向划分数量
    N_Z_LAYERS = 10;             % 侧面 Z 轴采样层数
    FRACTION_THRESHOLD = 1;  % 遮蔽比例阈值
    
    Num_Smokes = size(C_list, 1);
    if Num_Smokes == 0
        is_shielded = false;
        return;
    end
    
    % 1. 生成目标圆柱体采样点
    N_theta = N_THETA_DIVISIONS;
    thetas = linspace(0, 2*pi, N_theta + 1);
    thetas(end) = []; 
    
    X_circle = RT * cos(thetas)';
    Y_circle = RT * sin(thetas)';
    
    Samples_top = CB + [X_circle, Y_circle, HT * ones(N_theta, 1)];
    Samples_bottom = CB + [X_circle, Y_circle, zeros(N_theta, 1)];
    
    % 侧面采样
    z_coords = linspace(HT / (N_Z_LAYERS + 1), HT - HT / (N_Z_LAYERS + 1), N_Z_LAYERS);
    [Theta_M, Z_M] = meshgrid(thetas, z_coords);
    X_side = CB(1) + RT * cos(Theta_M);
    Y_side = CB(2) + RT * sin(Theta_M);
    Z_side = CB(3) + Z_M;
    Samples_side = [X_side(:), Y_side(:), Z_side(:)];
    
    % 结合所有采样点
    Target_Samples = [Samples_top; Samples_bottom; Samples_side];
    
    % 初始化 Master 命中数组 (Nx1 逻辑数组)
    is_hit_array = false(size(Target_Samples, 1), 1);

    % 2. 【核心修正】多烟幕向量化判定 (Union of Spheres)
    for j = 1:Num_Smokes
        C_j = C_list(j, :);
        
        % 判定当前烟幕对所有采样点的命中情况
        is_hit_j = segment_sphere_intersect_batch(PM, Target_Samples, C_j, R_eff);
        
        % 逻辑 OR 运算：只要被任一烟幕命中，就保持命中状态
        is_hit_array = is_hit_array | is_hit_j;
    end
    
    % 3. 比例判定
    Fraction_Hit = sum(is_hit_array) / size(Target_Samples, 1);
    
    if Fraction_Hit >= FRACTION_THRESHOLD
        is_shielded = true;
    else
        is_shielded = false;
    end
end