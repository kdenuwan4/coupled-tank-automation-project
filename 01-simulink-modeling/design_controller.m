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

stepinfo(T_P), stepinfo(T_PI)
dcgain(T_P), dcgain(T_PI)

%% PID design: pole-zero cancellation with filtered derivative
z = 0.007428;                          % plant pole location (rad/s), taken from the linearization
N = 0.05;                              % derivative filter pole (rad/s)
K = 0.03;                              % overall gain, chosen from the locus (poles stay real)
C0 = (s + z)^2 / (s*(s/N + 1));        % PID shape: two zeros on the plant poles, integrator, filter
C_PID = K*C0;
T_PID = feedback(C_PID*P, 1);

step(T_P, T_PI, T_PID); legend('P','PI','PID'); grid on
stepinfo(T_PID), dcgain(T_PID)