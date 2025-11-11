function P_M = get_MissilePos(M_P0, t, V_M, P_O)
% GET_MISSILEPOS 计算导弹在时刻 t 的位置。
% P_M = M_P0 + V_M * t * D_M

    D_M = P_O - M_P0;
    D_M = D_M / norm(D_M); 
    
    P_M = M_P0 + V_M * t * D_M;
end