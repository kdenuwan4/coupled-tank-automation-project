%% linearize_tanks.m
run('params.m');
load_system('TankModel');

io(1) = linio('TankModel/Step',  1, 'input');    % pump flow in
io(2) = linio('TankModel/Tank2', 1, 'output');   % Tank 2 level out

sys_lin = linearize('TankModel', io);

sys_lin                                  % display the state-space model
save('sys_lin.mat', 'sys_lin');          % save it for later steps

pole(sys_lin)
dcgain(sys_lin)
tf(sys_lin)

