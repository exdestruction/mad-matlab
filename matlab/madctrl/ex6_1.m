clear all;
%% Speed Controller

%% plant constants
P_ku = 2.51; % plant gain [m/s]
P_T = 316e-3; % plant time constant [s]
P_Tt = 0.1; %dead time [s]

num = P_ku;
den = [P_T 1];

%% controller parameters
P_Tw = 1000e-3; % close-loop time constant [ s ]
P_phires_arc = 65/360*2*pi; % phase margin [°]
P_omega_d = pi/P_Tw; %crossover frequency [rad/s]
P_Ta=0.02; %sample time [s]


P_Ti = (tan(P_phires_arc - pi + P_Tt*P_omega_d) * P_T * P_omega_d - 1)/...
    (P_T*P_omega_d^2 + tan(P_phires_arc - pi + P_Tt*P_omega_d) * P_omega_d );

P_kr = 1/P_ku * sqrt((P_Ti^2*P_omega_d^2 + P_T^2*P_Ti^2*P_omega_d^4) /...
    (1 + P_Ti^2*P_omega_d^2)); 


%% transfer functions
% plant 
Gs = tf(num,den,'InputDelay',P_Tt);

% controller
Gr = tf(P_kr * [P_Ti, 1 ], [ P_Ti , 0]);

% open-loop dynamics
G0 = Gr * Gs;

% closed-loop dynamics
Gw = G0 / (1 + G0);

%% plots
figure(1);
margin(G0);

figure(2);
stepplot(Gw);
grid();

S = stepinfo(Gw);