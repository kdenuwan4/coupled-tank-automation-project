
open_system('TankModel')   

% Physical parameters - lab-scale two-tank system
A1 = 0.02;      % Tank 1 cross-sectional area (m^2)
A2 = 0.02;      % Tank 2 cross-sectional area (m^2)
Cd1 = 0.6;      % Discharge coefficient, valve 1
Cd2 = 0.6;      % Discharge coefficient, valve 2
a1 = 0.5e-4;    % Valve 1 orifice area (m^2)
a2 = 0.5e-4;    % Valve 2 orifice area (m^2)
g = 9.81;       % gravity (m/s^2)

k1 = Cd1*a1*sqrt(2*g);   % lumped outflow coefficient, tank 1
k2 = Cd2*a2*sqrt(2*g);   % lumped outflow coefficient, tank 2

Qmax = 9e-5;
k_pump = Qmax/10;


% Target operating point
h2_ss = 0.2;                        % desired Tank 2 level
h1_ss = h2_ss;                      % forced equal, since k1 = k2

Q1_ss = k1*sqrt(h1_ss);              % flow out of Tank 1
Q2_ss = k2*sqrt(h2_ss);              % flow out of Tank 2
Qin_ss = Q1_ss;                      % at steady state, pump flow = Q1

u_ss = Qin_ss / k_pump;              % pump voltage needed for that flow


disp([h1_ss Q1_ss u_ss])

