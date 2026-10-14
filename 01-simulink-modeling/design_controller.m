%% design_controller.m - P and PI design on the linearized plant
load('sys_lin.mat');             % linearized plant from linearize_tanks.m
P = tf(sys_lin);                 % transfer function: 0.3714/(s+0.007428)^2
s = tf('s');

K  = 0.000277;                   % P gain read from root locus at ~10% overshoot
T_P = feedback(K*P, 1);          % closed loop with P only

z  = 0.005;                      % PI zero location (rad/s)
Kp = 0.000186;                    
C_PI = Kp*(s + z)/s;             % PI controller: Kp + Ki/s, with Ki = Kp*z
T_PI = feedback(C_PI*P, 1);      % closed loop with PI

step(T_P, T_PI); legend('P only', 'PI'); grid on
stepinfo(T_P), stepinfo(T_PI)
dcgain(T_P), dcgain(T_PI)