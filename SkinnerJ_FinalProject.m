% James Skinner
% ME 747 
% Final Project
close all; clear all;
%% Simulate PID Controlled RC car Response 

% Constants of the Motor

MIN_SPEED = 0; % m/s
MAX_SPEED = 0.858; % m/s
speeds = [0,.858]; % m/s
TIME_TO_MAX_VELOCITY = 2.8; % sec
times = [0,2.8]; % sec

tau = times(1)+0.632*(times(end)-times(1));

% % Using specs from http://www.robotcombat.com/products/0-B16.html
% % for voltage to omega to find Ke and convert to Kt using DC motor
% % assumption Ke = Kt
ei = [6,12]; % volt 
omega = [468,937]*(60/(2*pi)); % rad/sec 
% pfit = polyfit(omega,ei,1);
% Ke = pfit(1); % V/rad/sec
% Kt2 = Ke*141.6119; % (oz-in/A) torque constant

R = 40/2; % ohm, measured using voltmeter
%B = 0.002; % oz-in/rad/sec

%B = ((Kt/K)-(Ke*Kt))/R; % oz-in/rad/s
Ls = 0;
Td = interp1(ei,[27,45],7.2); % oz-in, stall torque for 7.2V input
Kt = Td*R/(7.2/4); % oz-in/A

g = 386; % in/sec^2, gravity
m = 2.29*2; % oz, weight of motor 
L = (4*.0393701)/2; % mm to in, distance from center of gravity of roatating shaft
J = (m*L^2)/g; % oz-in-sec^2

% PID Controller constants
Kp = 1; % P-Control
Ki = 1; % I-Control
Kd = 1; % D-Control

Target = 40; % Distance from the wall to stop

B = J/tau - Kt/R; % Damping of one motor



