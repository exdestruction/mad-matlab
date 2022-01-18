%% Clear workspace
clear all;

%% Constants
P_ku = 2.51; % plant gain [m/s]
P_T = 316e-3; % plant time constant [s]
P_Tt = 0.1; %dead time [s]

%% plant
%transfer function of plant
Gs = tf(P_ku, [ P_T, 1 ], 'ioDelay', P_Tt); 

%% plots
subplot(2,1,1);
bode(Gs);
grid on;
title("Bode-Diagram of GS(s)")
legend;

subplot(2,1,2);
step(Gs);
grid on;
title("Step Response of GS(s)")
legend;