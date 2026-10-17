%% stability.m - stability margins and gain sweep for the PID design
load('sys_lin.mat');                       % linearized plant
P = tf(sys_lin);
s = tf('s');

z = 0.007428;                              % plant pole location (rad/s)
N = 0.05;                                  % derivative filter pole (rad/s)
C0 = (s + z)^2 / (s*(s/N + 1));            % PID shape (gain K applied below)

L = C_PID*P;                      % open loop: controller in series with plant
margin(L)                         % Bode plot with margins marked
[Gm, Pm, Wcg, Wcp] = margin(L)    % the numbers themselves


for K = [0.03 0.1 1]
    L = K*C0*P;
    [~, Pm] = margin(L);
    T = feedback(L, 1);
    info = stepinfo(T);
    fprintf('K=%.2f  PM=%.1f deg  Overshoot=%.1f %%\n', K, Pm, info.Overshoot);
end