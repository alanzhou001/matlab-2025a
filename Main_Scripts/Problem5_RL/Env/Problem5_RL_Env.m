classdef Problem5_RL_Env < handle
    % Problem5_RL_Env (维度修复版)
    
    properties
        GlobalParams
        UAV_ActionDim   % 8
        Total_ActionDim % 40
        lb_one
        ub_one
        
        % 兼容属性
        ActionDim 
        lb
        ub
    end
    
    methods
        function obj = Problem5_RL_Env()
            obj.init_params();
            
            obj.UAV_ActionDim = 8;
            Num_UAVs = 5;
            obj.Total_ActionDim = Num_UAVs * obj.UAV_ActionDim;
            
            obj.ActionDim = obj.Total_ActionDim;
            
            % 单机边界 (8维)
            obj.lb_one = [0,    70,  0,  1,  1,  0, 0, 0];
            obj.ub_one = [2*pi, 140, 60, 10, 10, 10, 10, 10];
            
            % 全局边界 (40维)
            obj.lb = repmat(obj.lb_one, 1, Num_UAVs);
            obj.ub = repmat(obj.ub_one, 1, Num_UAVs);
        end
        

        function x_phys = denormalize_action(obj, action_norm)
            % 转为行向量
            if size(action_norm, 1) > 1, action_norm = action_norm'; end
            action_norm = max(min(action_norm, 1), -1);
            
            % 动态选择边界
            if length(action_norm) == obj.UAV_ActionDim
                % 单机模式 (8维)
                lower = obj.lb_one;
                upper = obj.ub_one;
            else
                % 多机模式 (40维)
                lower = obj.lb;
                upper = obj.ub;
            end
            
            range = upper - lower;
            
            % 计算 (使用 lower 而不是 obj.lb_one)
            x_phys = lower + (action_norm + 1) * 0.5 .* range;
        end

        % =================================================================
        % 协同奖励
        % 输入: action_norm_total (40维)
        % 输入: penalty_overlap (重叠惩罚系数)
        % 输入: penalty_lazy    (怠工惩罚系数)
        % =================================================================
        function [final_score, union_time, overlap_time, num_lazy] = get_cooperative_reward(obj, action_norm_total, penalty_overlap, penalty_lazy)
            
            All_Smokes = [];
            Individual_Times = zeros(5, 1);
            
            for i = 1:5
                idx_start = (i-1) * obj.UAV_ActionDim + 1;
                idx_end = i * obj.UAV_ActionDim;
                
                act_i = action_norm_total(idx_start:idx_end);
                x_phys_i = obj.denormalize_action(act_i);
                
                Smokes_i = obj.compute_smokes_for_uav(i, x_phys_i);
                All_Smokes = [All_Smokes; Smokes_i];
                
                % 计算单机贡献
                Individual_Times(i) = obj.calculate_shielding_time(Smokes_i);
            end
            
            % 1. 计算并集时长 (主要目标)
            union_time = obj.calculate_shielding_time(All_Smokes);
            
            % 2. 计算重叠时长
            sum_individual = sum(Individual_Times);
            overlap_time = sum_individual - union_time;
            
            % 3. 计算“怠工”无人机数量
            % 定义：如果某架飞机的贡献小于 0.5秒，视为“偷懒”或“无效”
            lazy_threshold = 0.5; 
            num_lazy = sum(Individual_Times < lazy_threshold);
            
            % 4. 最终得分计算
            % 逻辑：(并集收益) - (轻微的重叠惩罚) - (严重的怠工惩罚)
            final_score = union_time - (penalty_overlap * overlap_time) - (penalty_lazy * num_lazy);
        end

        function reward = get_individual_reward(obj, uav_idx, action_norm)
            % 这里 action_norm 是 8 维，denormalize 会自动识别
            x_phys = obj.denormalize_action(action_norm);
            Smokes = obj.compute_smokes_for_uav(uav_idx, x_phys);
            reward = obj.calculate_shielding_time(Smokes);
        end
        
        function reward = get_joint_reward(obj, action_norm_total)
            [reward, ~, ~] = obj.get_cooperative_reward(action_norm_total, 0);
        end
        
        function reward = step(obj, action_norm)
            reward = obj.get_joint_reward(action_norm);
        end

        function Smokes = compute_smokes_for_uav(obj, uav_idx, u_vars)
            P = obj.GlobalParams; Smokes = [];
            theta = u_vars(1); v = u_vars(2);
            t1 = u_vars(3); t2 = t1 + u_vars(4); t3 = t2 + u_vars(5);
            tau = u_vars(6:8);
            T_drop = [t1, t2, t3];
            V_Vec = v * [cos(theta), sin(theta), 0];
            Pos0 = P.UAVs(uav_idx).Pos0;
            
            for k = 1:3
                t_d = T_drop(k); delta = tau(k); t_det = t_d + delta;
                P_Drop = Pos0 + V_Vec * t_d;
                g_vec = [0, 0, -9.8];
                P_Det = P_Drop + V_Vec * delta + 0.5 * g_vec * delta^2;
                S.Pos = P_Det; S.Time = t_det;
                Smokes = [Smokes; S];
            end
        end
        
        function total_time = calculate_shielding_time(obj, Smokes)
            P = obj.GlobalParams; total_time = 0; dt = 0.5;
            for m = 1:3
                M0 = P.Missiles(m).Pos0; Dist = norm(M0 - P.Target_Fake);
                T_End = Dist / P.V_M; Dir = (P.Target_Fake - M0) / Dist;
                shield_count = 0; time_span = 0:dt:T_End;
                for t = time_span
                    PM = M0 + P.V_M * t * Dir;
                    Active_Smokes = [];
                    for k = 1:length(Smokes)
                        ts = Smokes(k).Time;
                        if t >= ts && t <= ts + P.Smoke_Duration
                            C = Smokes(k).Pos + [0, 0, -P.V_Sink * (t - ts)];
                            Active_Smokes = [Active_Smokes; C];
                        end
                    end
                    if ~isempty(Active_Smokes)
                        if obj.fast_check(PM, Active_Smokes, P.R_Smoke, P.T_Base), shield_count = shield_count + 1; end
                    end
                end
                total_time = total_time + shield_count * dt;
            end
        end
        
        function hit = fast_check(~, PM, C_list, R, T_Base)
            T_Center = T_Base + [0, 0, 5]; hit = false;
            for i = 1:size(C_list, 1)
                C = C_list(i, :);
                U = T_Center - PM; AC = C - PM;
                proj = dot(AC, U) / (dot(U, U) + eps); proj = max(0, min(1, proj));
                Closest = PM + proj * U;
                if norm(C - Closest) <= R, hit = true; return; end
            end
        end
        
        function init_params(obj)
            obj.GlobalParams.T_Base = [0, 200, 0];
            obj.GlobalParams.Target_Fake = [0, 0, 0];
            obj.GlobalParams.Missiles(1).Pos0 = [20000, 0, 2000];
            obj.GlobalParams.Missiles(2).Pos0 = [19000, 600, 2100];
            obj.GlobalParams.Missiles(3).Pos0 = [18000, -600, 1900];
            obj.GlobalParams.V_M = 300; 
            obj.GlobalParams.UAVs(1).Pos0 = [17800, 0, 1800];
            obj.GlobalParams.UAVs(2).Pos0 = [12000, 1400, 1400];
            obj.GlobalParams.UAVs(3).Pos0 = [6000, -3000, 700];
            obj.GlobalParams.UAVs(4).Pos0 = [11000, 2000, 1800];
            obj.GlobalParams.UAVs(5).Pos0 = [13000, -2000, 1300];
            obj.GlobalParams.V_Sink = 3;
            obj.GlobalParams.R_Smoke = 10;
            obj.GlobalParams.Smoke_Duration = 20;
        end
    end
end