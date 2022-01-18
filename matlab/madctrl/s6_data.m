%
% Mini-Auto-Drive
%
% System Parameters
%
% Copyright (c) 2020 N. Heining, Uwe Ingelfinger, Frank Traenkle
% http://www.modbas.de
%
% This file is part of Mini-Auto-Drive.
%
% Mini-Auto-Drive is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% Mini-Auto-Drive is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.
%
%% s6.data has been extended to s7.data by Paul Wolff and Verena Barske
% Implementation of solutions for Ex. 6.1, 6.2, 8.1, 8.2, 9.1 
% in course context Autonomous Systems - Path Planning and Control
% Wintersemester 2020/21

%% Clear workspace
clear all;

%% Global Variables
global P_dt P_display_dt;

%% Car and Belt Configuration
p_mad_car;

%% Sample Times
P_dt = 20e-3; % sample time of controller
P_sim_dt = 2e-3; % sample time of simulation
P_display_dt = 40e-3; % sample time of display

%% plant parameters
P_Tt = 0.1; %dead time [s]
P_T = 316e-3; % plant time constant [s]
P_ku = 2.51; % plant gain [m/s]


%% Speed Controller parameters

% motor voltage limits
P_un_max=1;
P_un_min=-1;

%Target speed [m/s]
P_vTarget = 1.5;

%PI controller design
P_Tm = 1000e-3; % close-loop time constant [s]
P_omega_d = pi/P_Tm; %crossover frequency [rad/s]
P_phi_res = 2*pi*65/360; % phase margin [°]

P_Ta=0.02; %sample time controller[s]

% integral time constant by dynamic compensation [ s ]
P_Ti = tan(P_phi_res - pi/2 + P_omega_d*P_Tt + atan(P_omega_d*P_T)) / ...
    P_omega_d;

% calculation of controller gain [ s/m ]
P_kr = sqrt((P_omega_d^2*P_T*P_Ti)^2 + (P_omega_d*P_Ti)^2) / ...
    (sqrt(1+(P_omega_d*P_Ti)^2) * P_ku);


%% Lateral Controller

P_Tw = 300e-3; %Time constant of FeedbackController
P_delta_n_max=1;
P_delta_n_min=-1;
P_v_fb_border = 0.1; %[m/s] singularity eliminator in Feed-back controller

%% Create Buga track
a1total = 2.7; % total surface width [ m ]
a2total = 1.8; % total surface height [ m ]
a1boundary = 0.05; % margin [ m ]
a2boundary = 0.05; % margin [ m ]
a1 = a1total - 2 * a1boundary; % total surface width [ m ]
a2 = a2total - 2 * a2boundary; % total surface height [ m ]
P_width = 0.2; % track P_width [ m ]
P_cloth_a=8; %clothoid parameter
opening=1; %opening clothoid
closing=0; %closing clothoid

track = mbc_track_create(a1boundary + P_width/2, a2total/2, -pi/2);

% straight 2 + corner 1
track = mbc_straight_create(track, (a2total/2 - a2boundary - P_width - 0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

% straight 2 + corner 2
track = mbc_straight_create(track, (a1total - 2*a1boundary - 2*P_width - 2*0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

% straight 3 + corner 3
track = mbc_straight_create(track, (a2total - 2*a2boundary - 2*P_width - 2*0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

% straight 4 + corner 4
track = mbc_straight_create(track, (a1total - 2*a1boundary - 2*P_width - 2*0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

% straight 5
track = mbc_straight_create(track, (a2total/2 - a2boundary - P_width - 0.4275), P_width);

track = mbc_track_display(track, 0.1, [ 0 a1total 0 a2total ]);
path = track.center;


%% Choose car starting position

P_p_s10 = 1;  % s1 position [ m ] Carid 0
P_p_s20 = 1;  % s2 position [ m ] Carid 0
P_p_psi0 = 0; % yaw angle [ rad ] 
    

%% Workspace variables for reference track generation in Simulink
P_w_breakslen = uint32(length(path.points));
P_w_points = zeros(SPLINE.Elements(2).Dimensions); 
P_w_points(:,1:length(path.points)) = path.points;
P_w_coefs = zeros(SPLINE.Elements(3).Dimensions);
P_w_coefs(1:length(path.pp.coefs),:) = path.pp.coefs;
P_w_segments = uint32(zeros(SPLINE.Elements(4).Dimensions)); 

% Init car display
mbc_car_display(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);


