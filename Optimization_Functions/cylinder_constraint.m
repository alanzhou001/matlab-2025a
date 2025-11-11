function [c, ceq] = cylinder_constraint(PT, RT, CB, HT)
% CYLINDER_CONSTRAINT 定义圆柱体边界约束。
% fmincon 要求非线性不等式约束 c <= 0。

    % 1. 圆周约束 (水平面): x^2 + (y - 200)^2 <= R_T^2
    c1 = (PT(1))^2 + (PT(2) - CB(2))^2 - RT^2;
    
    % 2. 高度约束: 0 <= z <= HT
    c2 = -PT(3); % z >= 0  => -z <= 0
    c3 = PT(3) - HT; % z <= HT => z - HT <= 0
    
    c = [c1; c2; c3];
    ceq = [];
end