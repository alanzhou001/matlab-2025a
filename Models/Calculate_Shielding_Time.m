function total_time = Calculate_Shielding_Time(x, P)
    % 解包决策变量
    theta  = x(1);
    v_d    = x(2);
    t_drop = x(3);
    tau    = x(4);
    
    t_det = t_drop + tau; % 起爆时刻
    
    % 1. 计算起爆点 P_det
    V_UAV_Vec = v_d * [cos(theta), sin(theta), 0];
    P_Drop_Pos = P.UAV_Pos0 + V_UAV_Vec * t_drop;
    
    % 调用你的 get_SmokeDetPos
    P_det = get_SmokeDetPos(P_Drop_Pos, t_drop, V_UAV_Vec, t_det);
    
    % 2. 时间扫描设置
    Dist_M_Total = norm(P.M0 - P.Target_Fake);
    T_Missile_End = Dist_M_Total / P.V_M;
    
    T_start = t_det;
    T_end = min(t_det + P.Smoke_Duration, T_Missile_End);
    
    if T_end <= T_start
        total_time = 0;
        return;
    end
    
    dt = 0.05; 
    time_steps = T_start : dt : T_end;
    shield_count = 0;
    
    D_M_Dir = (P.Target_Fake - P.M0) / norm(P.Target_Fake - P.M0);
    
    for t = time_steps
        % 导弹位置
        PM = P.M0 + P.V_M * t * D_M_Dir;
        
        % 烟幕中心 (下沉)
        C_smoke = P_det + [0, 0, -P.V_Sink * (t - t_det)];
        
        % 遮蔽判定
       
        is_covered = is_FullyShielded(PM, C_smoke, P.R_Smoke, P.R_T, P.H_T, P.T_Bottom);
        
        if is_covered
            shield_count = shield_count + 1;
        end
    end
    
    total_time = shield_count * dt;
end