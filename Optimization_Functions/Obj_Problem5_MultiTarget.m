function F_neg = Obj_Problem5_MultiTarget(x, P)
% 输入 x: 40维向量
% 输出 F_neg: 负的总遮蔽时长 (M1时长 + M2时长 + M3时长)

    Num_UAVs = 5;
    Num_Missiles = 3;
    
    % --- 1. 解码所有烟幕事件 ---
    % Smokes 结构体数组: .Pos (起爆位置), .Time (起爆时间)
    All_Smokes = [];
    
    for i = 1:Num_UAVs
        idx = (i-1)*8 + 1;
        u_vars = x(idx : idx+7);
        
        theta = u_vars(1);
        v = u_vars(2);
        t1 = u_vars(3);
        t2 = t1 + u_vars(4); % 增量解码
        t3 = t2 + u_vars(5);
        tau = u_vars(6:8);
        
        T_drop_list = [t1, t2, t3];
        
        V_UAV_Vec = v * [cos(theta), sin(theta), 0];
        
        for k = 1:3
            t_d = T_drop_list(k);
            t_det = t_d + tau(k);
            
            % 计算起爆位置
            P_Drop = P.UAVs(i).Pos0 + V_UAV_Vec * t_d;
            % 简化的抛体计算 (直接嵌入以提速)
            g_vec = [0, 0, -9.8];
            delta_t = tau(k);
            P_Det = P_Drop + V_UAV_Vec * delta_t + 0.5 * g_vec * delta_t^2;
            
            Smoke.Pos = P_Det;
            Smoke.Time = t_det;
            All_Smokes = [All_Smokes; Smoke];
        end
    end
    
    % --- 2. 对每枚导弹计算遮蔽时长 ---
    Total_Time = 0;
    dt = 0.1; % 时间步长
    
    for m = 1:Num_Missiles
        M_Pos0 = P.Missiles(m).Pos0;
        
        % 估算该导弹飞行结束时间 (撞击原点)
        Dist = norm(M_Pos0 - P.Target_Fake);
        T_End = Dist / P.V_M;
        
        % 扫描时间轴
        t_span = 0:dt:T_End;
        shield_count = 0;
        
        % 导弹方向向量
        Dir_M = (P.Target_Fake - M_Pos0) / Dist;
        
        for t = t_span
            PM_t = M_Pos0 + P.V_M * t * Dir_M;
            
            % 筛选当前有效的烟幕中心
            Current_C_List = [];
            for s = 1:length(All_Smokes)
                t_s = All_Smokes(s).Time;
                if t >= t_s && t <= t_s + P.Smoke_Duration
                    % 计算下沉后的中心
                    C_sink = All_Smokes(s).Pos + [0, 0, -P.V_Sink * (t - t_s)];
                    Current_C_List = [Current_C_List; C_sink];
                end
            end
            
            % 遮蔽判定
            if ~isempty(Current_C_List)
                % 调用现有的判定函数 (需确保在路径中)
                % 注意: is_FullyShielded 需要针对新的参数结构适配，或者使用向量化版本
                if is_FullyShielded_Fast(PM_t, Current_C_List, P.R_Smoke, P.R_T, P.H_T, P.T_Bottom)
                    shield_count = shield_count + 1;
                end
            end
        end
        
        Total_Time = Total_Time + shield_count * dt;
    end
    
    F_neg = -Total_Time;
end

% 这是一个为了速度优化的简化判定版本，你可以直接用提供的segment_sphere_intersect_batch
function is_shielded = is_FullyShielded_Fast(PM, C_list, R_smoke, RT, HT, CB)
    % 简易判定：只要真目标中心被挡住，或者关键点被挡住
    % 这里为了代码独立性，写一个简单的 单射线检测
    % 实际比赛请使用之前提供的严谨的 is_FullyShielded
    
    Target_Center = CB + [0, 0, HT/2];
    is_shielded = false;
    
    for i = 1:size(C_list, 1)
        C = C_list(i,:);
        % 计算点到线段距离
        % 线段: PM -> Target_Center
        U = Target_Center - PM;
        AC = C - PM;
        proj = dot(AC, U) / dot(U, U);
        proj = max(0, min(1, proj));
        Closest = PM + proj * U;
        Dist = norm(C - Closest);
        
        if Dist <= R_smoke
            is_shielded = true;
            return;
        end
    end
end