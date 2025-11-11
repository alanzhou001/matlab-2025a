function is_hit = segment_sphere_intersect_batch(M, Q_matrix, C, R)
% 向量化判定线段 M->Q 是否与球 S(C,R) 相交

    U = Q_matrix - M; % 视线向量 U = Q - M (Nx3)
    
    % 向量化计算投影点 s = ( (C-M) . U ) / (U . U)
    uu = sum(U .* U, 2); % ||U||^2 (Nx1)
    CM = C - M; % C - M (1x3)
    s = sum(CM .* U, 2) ./ uu; % (Nx1)
    
    % 截断 s 到 [0, 1] 确保投影点在线段上
    s_clip = max(0, min(1, s)); % (Nx1)
    
    % 计算最近点 Closest = M + s_clip * U
    % 使用 bsxfun 进行逐行乘法
    Closest = M + bsxfun(@times, s_clip, U); % (Nx3)
    
    % 计算距离 D = ||Closest - C||
    D_sq = sum((Closest - C) .^ 2, 2); % 距离的平方 (Nx1)
    
    is_hit = D_sq <= R^2; % 布尔数组 (Nx1)
end