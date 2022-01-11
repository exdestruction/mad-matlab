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

%% EKF e6
%e6_data;

%% Speed Controller
%% plant
k = 2.51; % plant gain [ m/s ]
T = 316e-3; % plant time constant [ s ]
Gs = tf(k, [ T , 1 ]);

%% controller parameters
Tw = 1000e-3; % close-loop time constant [ s ]
Ti = T; % integral time constant by dynamic compensation [ s ]
kp = T / (Tw * k); % controller gain [ s/m ]
Gr = tf(kp * [Ti, 1 ], [ Ti , 0]);

%% open-loop dynamics
G0 = minreal (Gr * Gs);

%% closed-loop dynamics
Gw = G0 / (1 + G0);


%% parameters
delta_n = 0.7;
v_max = 1;
P_kr = 0.3345;
P_Ta = 0.1;
P_Ti = 0.2474;
P_Tt = T;
un_max = 1;
un_min = -1;




%% Lateral Controller
%% TODO

%% Create Race Track
a1total = 2.7; % total surface width [ m ]
a2total = 1.8; % total surface height [ m ]
a1boundary = 0.05; % margin [ m ]
a2boundary = 0.05; % margin [ m ]
a1 = a1total - 2 * a1boundary; % total surface width [ m ]
a2 = a2total - 2 * a2boundary; % total surface height [ m ]
P_width = 0.25 * a2; % track P_width [ m ]

track = mbc_track_create(a1boundary + P_width, a2boundary + 0.5 * P_width, 0);
track = mbc_straight_create(track, a1 - 2 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, pi, P_width);
track = mbc_straight_create(track, a1 - 3 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, -pi, P_width);
track = mbc_straight_create(track, a1 - 3 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, pi, P_width);
track = mbc_straight_create(track, a1 - 2 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, 0.5 * pi, P_width);
track = mbc_straight_create(track, a2 - 2 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, 0.5 * pi, P_width);
track = mbc_track_display(track, 0.1, [ 0 a1total 0 a2total ]);
path = track.center;

%% Workspace variables for reference track generation in Simulink
P_w_breakslen = uint32(length(path.points));
P_w_points = zeros(SPLINE.Elements(2).Dimensions); 
P_w_points(:,1:length(path.points)) = path.points;
P_w_coefs = zeros(SPLINE.Elements(3).Dimensions);
P_w_coefs(1:length(path.pp.coefs),:) = path.pp.coefs;
P_w_segments = uint32(zeros(SPLINE.Elements(4).Dimensions)); 

% Init car display
mbc_car_display(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
