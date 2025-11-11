function C_j = get_Smokecenter(P_det, t_det, t, V_sink)
% GET_SMOKECENTER 计算烟幕云团在时刻 t 的中心位置 (匀速下沉)。

    if t < t_det
        C_j = P_det; 
    else
        Delta_t = t - t_det;
        D_sink = [0, 0, -V_sink * Delta_t]; 
        C_j = P_det + D_sink;
    end
end