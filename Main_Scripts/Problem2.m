clear; clc; close all;

% --- 1. 参数设置 ---
GlobalParams.M0 = [20000, 0, 2000];   
GlobalParams.Target_Fake = [0, 0, 0]; 
GlobalParams.V_M = 300;               
GlobalParams.T_Bottom = [0, 200, 0];  
GlobalParams.R_T = 7;                 
GlobalParams.H_T = 10;                
GlobalParams.UAV_Pos0 = [17800, 0, 1800]; 
GlobalParams.V_Sink = 3;              
GlobalParams.R_Smoke = 10;            
GlobalParams.Smoke_Duration = 20;     

% --- 2. 变量边界 
% 缩小时间范围，强迫粒子在有效区域搜索
lb = [0,       70,   0,   0];     
ub = [2*pi,    140,  15,  5];    

% --- 3. 算法设置 ---
pso_options.SwarmSize = 100;      
pso_options.MaxIter = 120;        
pso_options.c1 = 1.5;
pso_options.c2 = 1.5;

ObjFcn = @(x) -Calculate_Shielding_Time(x, GlobalParams);

fprintf('正在启动多轮搜索 (共5轮)...\n');

% === 多轮择优 ===
Num_Runs = 5;          
Best_Global_Val = 0;   % 初始化为0 
Max_Duration_Record = -1; 
Best_Global_X = zeros(1, 4);
Best_History = zeros(pso_options.MaxIter, 1);

for run = 1:Num_Runs
    fprintf('>>> 第 %d / %d 轮搜索... ', run, Num_Runs);
    
    % 调用算法
    [x_tmp, fval_tmp, history_tmp] = run_simple_pso(ObjFcn, 4, lb, ub, pso_options);
    
    % 计算真实时长 (取反)
    current_duration = -fval_tmp;
    fprintf('本轮结果: %.4f 秒\n', current_duration);
    
    % 擂台赛
    if current_duration > Max_Duration_Record
        Max_Duration_Record = current_duration;
        Best_Global_X = x_tmp;
        Best_History = history_tmp;
    end
end

% --- 4. 结果输出 ---
if Max_Duration_Record <= 0.0001
    fprintf('\n警告：算法未找到有效解 (结果为0)。\n');
else
    fprintf('\n最终结果\n');
    fprintf('最优遮蔽时长: %.4f 秒\n', Max_Duration_Record);
    fprintf('------------------------------------------\n');
    fprintf('最优策略参数:\n');
    fprintf('航向角 (theta):   %.2f 度\n', rad2deg(Best_Global_X(1)));
    fprintf('飞行速度 (v_d):    %.2f m/s\n', Best_Global_X(2));
    fprintf('投放时刻 (t_drop): %.4f s\n', Best_Global_X(3));
    fprintf('起爆延时 (tau):    %.4f s\n', Best_Global_X(4));
    
    % 绘图 
    if ~isempty(Best_History)
        figure;
        plot(-Best_History, 'LineWidth', 2);
        xlabel('迭代次数'); ylabel('遮蔽时长 (秒)');
        title(['最优收敛曲线 (基于 ' num2str(Num_Runs) ' 轮搜索)']);
        grid on;
    end
end