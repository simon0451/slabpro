% James Skinner 
% final Project 
% root locus to tune 
close all; clear all;
%% Root locus

J = 0.1306;
b = 4.5905;
K = 0.9719;
R = 4;
L = 0;
tau = 0.0244; 
s = tf('s');
P_motor = K/(s*((J*s+b)*(L*s+R)+K^2));
C = (s^2 + 0.4*s + 0.05)/s;
poles = pole(P_motor);
rP_motor = minreal(P_motor*(s/max(abs(poles)) + 1));
rsys_ol = minreal(C*rP_motor, 0.1);
%pole(P_motor)
%Vtau = 0.632*(2.7);

figure
rlocus(C*rP_motor)
title('Root Locus - PID Control')
sgrid(.5, 0)
sigrid(1/2.7)
xlim([-0.5 0.2])

%[k,poles] = rlocfind(rsys_ol)
k = .9719;
sys_cl = feedback(k*rsys_ol,10);
t = 0:0.01:5;

figure
step(sys_cl, t)
grid
ylabel('Position (cm)')
title('Response to a Step Reference with PID Control')

% at 0.4 and 5 it stopped at 20 cm
% at 0.6 and 6 it stopped at 17 cm
% at 0.8 and 8 it stopped at 14 cm
% at 1 and 10 it stopped at 11 cm
