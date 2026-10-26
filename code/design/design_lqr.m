%% design_lqr.m — 平衡车 LQR 设计脚本（Octave / MATLAB 通用）
% 输入: 第3.4节实测参数; 输出: 写入 balance_config.h 的 K
% 详细推导见 docs/双轮自平衡机器人-毕业设计完整方案与答辩手册.docx 第4章
clear; close all;

%% ---- 实测机械参数(摆线法/吊线法/两工况法, 见手册3.4节) ----
Mp = 1.5;  L = 0.10;  Jp = 0.005;    % 车体质量kg / 质心高m / 绕质心惯量kg·m²
m  = 0.05; r = 0.03;                 % 单轮质量kg / 半径m
g  = 9.8;

%% ---- 派生量 ----
Jw  = 0.5*m*r^2;                     % 车轮转动惯量
Ib  = Jp + Mp*L^2;                   % 车体绕轮轴惯量(平行轴定理)
Meq = Mp + 2*(Jw + m*r^2)/r^2;       % 等效质量(车轮折算)
D   = Ib*Meq - Mp^2*L^2;             % 判别式, 必须大于0
assert(D > 0, 'D<=0: 方程组退化, 检查参数');

%% ---- 状态空间: x=[θ, θ̇, s, v], u=双轮合计力矩T ----
a1 =  Meq*Mp*g*L/D;   b1 = -2*Mp*L/(r*D);
a2 = -Mp^2*L^2*g/D;   b2 =  2*Ib/(r*D);
A = [0 1 0 0; a1 0 0 0; 0 0 0 1; a2 0 0 0];
B = [0; b1; 0; b2];

fprintf('a1=%.1f b1=%.1f a2=%.1f b2=%.1f\n', a1, b1, a2, b2);
fprintf('开环不稳定极点: ±%.2f rad/s -> 1°倒到10°需 %.0f ms\n', ...
        sqrt(a1), acosh(10)/sqrt(a1)*1000);

%% ---- 能控性(=4才有解) ----
rank(ctrb(A,B))

%% ---- LQR 求解 ----
Q = diag([50 2 1 100]);   R = 1;     % 起点权重, 整定方向见手册表4-2
K = lqr(A, B, Q, R)
% 期望输出 K ≈ [-2.798 -0.326 -0.094 -0.971] (θ 单位 rad, 复制进 balance_config.h)

%% ---- 校核 ----
disp('闭环极点:'); disp(eig(A - B*K))       % 全部实部为负
figure; step(ss(A - B*K, eye(4), eye(4), zeros(4)), 3);
title('闭环对4个初始状态的响应');
