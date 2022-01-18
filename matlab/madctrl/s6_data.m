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

%% Speed Controller parameters
% plant
P_ku = 2.51; % plant gain [m/s]
P_T = 316e-3; % plant time constant [s]
P_Tt = 0.1; %dead time of motorelectronics and transmission [s]

%PI controller design
P_Tm = 1000e-3; % close-loop time constant [s]
P_phires_arc = 65/360*2*pi; % phase margin [°]
P_omega_d = pi/P_Tm; %crossover frequency [rad/s]
P_Ta=0.02; %sample time [s]

% integral time constant by dynamic compensation [ s ]
P_Ti = (tan(P_phires_arc - pi + P_Tt*P_omega_d) * P_T * P_omega_d - 1)/...
    (P_T*P_omega_d^2 + tan(P_phires_arc - pi + P_Tt*P_omega_d) * P_omega_d ); 
% controller gain [ s/m ]
P_kr = 1/P_ku * sqrt(P_Ti^2*P_omega_d^2 + P_T^2*P_Ti^2*P_omega_d^4) /...
    sqrt(1 + P_Ti^2*P_omega_d^2); 

%Limitation of motor voltage to [-1...1]
P_un_max=1;
P_un_min=-1;

%Target speed [m/s]
P_vTarget = 1.5;

%% Lateral Controller
%% 
P_Tw = 300e-3; %Time constant of FeedbackController
P_dn_max=1;
P_dn_min=-1;

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

track = mbc_straight_create(track, (a2total/2 - a2boundary - P_width - 0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

track = mbc_straight_create(track, (a1total - 2*a1boundary - 2*P_width - 2*0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

track = mbc_straight_create(track, (a2total - 2*a2boundary - 2*P_width - 2*0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

track = mbc_straight_create(track, (a1total - 2*a1boundary - 2*P_width - 2*0.4275), P_width);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, closing);
track = mbc_clothoid_create(track, P_cloth_a, pi/4, P_width, opening);

track = mbc_straight_create(track, (a2total/2 - a2boundary - P_width - 0.4275), P_width);

track = mbc_track_display(track, 0.1, [ 0 a1total 0 a2total ]);
path = track.center;

%% Path for Lap Statistics
lappath = track.center;
P_lap_breakslen = uint32(length(lappath.points));
P_lap_points = zeros(SPLINE.Elements(2).Dimensions); 
P_lap_points(:,1:length(lappath.points)) = lappath.points;
P_lap_coefs = zeros(SPLINE.Elements(3).Dimensions);
P_lap_coefs(1:length(lappath.pp.coefs),:) = lappath.pp.coefs;
P_lap_segments = uint32(zeros(SPLINE.Elements(4).Dimensions)); 

%% Workspace variables for reference track generation in Simulink
P_w_breakslen = uint32(length(path.points));
P_w_points = zeros(SPLINE.Elements(2).Dimensions); 
P_w_points(:,1:length(path.points)) = path.points;
P_w_coefs = zeros(SPLINE.Elements(3).Dimensions);
P_w_coefs(1:length(path.pp.coefs),:) = path.pp.coefs;
P_w_segments = uint32(zeros(SPLINE.Elements(4).Dimensions)); 

% Init car display
mbc_car_display(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);


