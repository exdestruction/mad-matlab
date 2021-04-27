%%
%% Modbas - Frank Traenkle
%%
%% Parameters for Simulink model pt1_blockdiagram.slx
%%

clear variables; % clear workspace

P_T = 100e-6; % time constant [ s ]
P_k = 1;      % gain [ 1 ]
P_x0 = 0;     % initial value of capacity voltage [ V ]
P_dt = 10e-6; % step size of ODE solver [ s ]   
P_Ta = 5e-6;  % sample time of discrete PT1 [ s ]