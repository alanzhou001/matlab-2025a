clear; clc; close all;

% --- 1. 锁定最优策略 ---
best_theta_deg = 4.40; 
best_v_d       = 90.43;
best_t_drop    = 1.44;
best_tau       = 0.00;

% 转换为优化向量 x = [theta(rad), v_d, t_drop, tau]
x_opt = [deg2rad(best_theta_deg), best_v_d, best_t_drop, best_tau];

fprintf('正在评估最优策略的鲁棒性...\n');
fprintf('当前基准策略参数:\n');
fprintf('  > 航向角: %.2f 度\n', best_theta_deg);
fprintf('  > 速度:   %.2f m/s\n', best_v_d);
fprintf('  > 投放:   %.2f s\n', best_t_drop);
fprintf('  > 延时:   %.2f s\n', best_tau);

% --- 2. 原始环境参数设置  ---
BaseParams.M0 = [20000, 0, 2000];   
BaseParams.Target_Fake = [0, 0, 0]; 
BaseParams.V_M = 300;               
BaseParams.T_Bottom = [0, 200, 0];  
BaseParams.R_T = 7;                 
BaseParams.H_T = 10;                
BaseParams.UAV_Pos0 = [17800, 0, 1800]; 
BaseParams.V_Sink = 3;              
BaseParams.R_Smoke = 10;            
BaseParams.Smoke_Duration = 20;     

% --- 3. 蒙特卡洛模拟设置 ---
N_Sim = 1000; % 模拟 1000 次 
Results_Time = zeros(N_Sim, 1);

% 定义扰动范围
Pos_Error_Max = 5;  % 雷达定位误差 +/- 5m (X, Y, Z方向)
Vel_Error_Range = [298, 302]; % 导弹速度波动范围 m/s (原速300)

fprintf('\n开始进行 %d 次蒙特卡洛仿真 (Monte Carlo Simulation)...\n', N_Sim);

% 开启进度条
h = waitbar(0, '正在进行鲁棒性测试...');

for i = 1:N_Sim
    % 每次循环都重置为基准参数
    CurrentParams = BaseParams;
    
    % --- 施加随机扰动  ---
    
    % 1. 导弹初始位置误差 (模拟雷达精度)
    noise_pos = (rand(1, 3) - 0.5) * 2 * Pos_Error_Max; 
    CurrentParams.M0 = BaseParams.M0 + noise_pos;
    
    % 2. 导弹速度波动 (模拟发动机推力不稳或气流影响)
    CurrentParams.V_M = Vel_Error_Range(1) + ...
        (Vel_Error_Range(2) - Vel_Error_Range(1)) * rand();
    
    % --- 计算当前恶劣环境下的遮蔽时长 ---
    Results_Time(i) = Calculate_Shielding_Time(x_opt, CurrentParams);
    
    % 更新进度条
    if mod(i, 100) == 0
        waitbar(i/N_Sim, h, sprintf('进度: %d/%d', i, N_Sim));
    end
end
close(h);

% --- 4. 数据统计分析 ---
Mean_Time = mean(Results_Time);
Std_Time = std(Results_Time);
Min_Time = min(Results_Time);
Max_Time = max(Results_Time);

% 计算有效率：假设遮蔽时长大于 1秒 就算由于干扰虽然性能下降但依然"任务成功"
Success_Threshold = 1.0; 
Success_Rate = sum(Results_Time > Success_Threshold) / N_Sim * 100;

% --- 5. 结果输出与打印 ---
fprintf('\n========== 鲁棒性分析报告 ==========\n');
fprintf('模拟次数: %d\n', N_Sim);
fprintf('干扰条件: 位置误差 ±%dm, 速度 %.0f-%.0f m/s\n', Pos_Error_Max, Vel_Error_Range(1), Vel_Error_Range(2));
fprintf('------------------------------------\n');
fprintf('基准时长 (无干扰): 4.6000 秒\n');
fprintf('平均时长 (有干扰): %.4f 秒\n', Mean_Time);
fprintf('最差情况:         %.4f 秒\n', Min_Time);
fprintf('最好情况:         %.4f 秒\n', Max_Time);
fprintf('标准差:           %.4f\n', Std_Time);
fprintf('有效任务率 (>1s):  %.2f%%\n', Success_Rate);
fprintf('====================================\n');

% --- 6. 绘图---
figure('Color', 'w', 'Position', [100, 100, 900, 600]);

% 1. 绘制直方图 
h = histogram(Results_Time, 40, 'Normalization', 'pdf', ...
    'FaceColor', [0.85, 0.85, 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;

% 2. 绘制核密度估计曲线 
[f, xi] = ksdensity(Results_Time);
plot(xi, f, 'Color', [0, 0.45, 0.74], 'LineWidth', 2.5);

% 3. 标注 95% 置信区间
CI_Lower = quantile(Results_Time, 0.05); % 5% 分位数
x_fill = xi(xi >= CI_Lower);
y_fill = f(xi >= CI_Lower);
fill([x_fill, fliplr(x_fill)], [y_fill, zeros(size(y_fill))], ...
    [0, 0.45, 0.74], 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% 4. 辅助线
xline(4.60, 'Color', [0.47, 0.67, 0.19], 'LineWidth', 2, 'LineStyle', '--');
text(4.58, max(f)*0.9, '理论极限 (4.6s)', 'Color', [0.47, 0.67, 0.19], ...
    'HorizontalAlignment', 'right', 'FontSize', 12, 'FontWeight', 'bold');

xline(Mean_Time, 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2, 'LineStyle', '-.');
text(Mean_Time-0.05, max(f)*0.95, sprintf('实战平均 (%.2fs)', Mean_Time), ...
    'Color', [0.85, 0.33, 0.1], 'HorizontalAlignment', 'right', 'FontSize', 12);

% 5. 装饰
xlabel('有效遮蔽时长 (秒)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('概率密度 (Probability Density)', 'FontSize', 14, 'FontWeight', 'bold');
title('最优策略在干扰环境下的性能分布 (Robustness Profile)', 'FontSize', 16);
legend({'仿真分布直方图', '概率密度曲线 (KDE)', '95% 高置信区', '理论最优值', '均值'}, ...
    'Location', 'northwest', 'FontSize', 11);
grid on; set(gca, 'LineWidth', 1.2, 'FontSize', 12);
ylim([0, max(f)*1.2]); 