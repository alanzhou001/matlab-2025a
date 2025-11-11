function P_det = get_SmokeDetPos(P_drop, t_drop, V_UAV, t_det)
% GET_SMOKEDETPOS 计算烟幕弹的起爆点位置 (抛体运动)。

    g = [0, 0, -9.8]; % 重力加速度向量 (m/s^2)
    Delta_t = t_det - t_drop;
    
    if Delta_t < 0
        error('t_det 必须大于 t_drop.');
    end
    
    % P_det = P_drop + V_UAV * Delta_t + 0.5 * g * Delta_t^2
    P_det = P_drop + V_UAV * Delta_t + 0.5 * g * (Delta_t^2);
end