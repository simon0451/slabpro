% James Skinner
% ME 747 
% Final Project
close all; clear all;
%% Experimental data With Load Full Speed

nheaderlines = 31;
dataexp = importdata('Motors on table with wires.lvm','\t',nheaderlines);

time = dataexp.data(:,1);
time = time(3146:end);
time = time-time(1);
volt = dataexp.data(:,2);
volt = volt(3146:end);
voltnew = volt;
timenew = time;

j = 1;
for i = 1:length(time)
    holder = volt(i);
    if (holder>(-6) || holder<(-7.27))
       indexes(j) = i;
       j = j+1;
    end
    
end
voltnew(indexes) = [];
timenew(indexes) = [];
voltnew = voltnew(1:577);
voltnew(218) = [];
voltnew(411) = [];
voltnew(429) = [];
timenew = timenew(1:577);
timenew(218) = [];
timenew(411) = [];
timenew(429) = [];

Vtau = voltnew(1)+0.632*(voltnew(end)-voltnew(1));
indexVtau = find(voltnew>Vtau-.02 & voltnew<Vtau+.02);
tau = mean(time(indexVtau)); 

figure
plot(timenew,voltnew,tau,Vtau,'ro')
xlabel('Time (sec)')
ylabel('Voltage (V)')
title('Output Voltage With Load at Full speed')
legend('Experimental Data','Time Constant, \tau')
set(gca,'fontname','Times','fontsize',12)
xlim([0 0.4])
grid on
box on

voltsmooth = smooth(voltnew);

figure
plot(timenew,voltsmooth,tau,Vtau,'ro')
xlabel('Time (sec)')
ylabel('Voltage (V)')
title('Output Voltage With Load at Full Speed')
legend('Smoothed Experimental Data','Time Constant, \tau')
set(gca,'fontname','Times','fontsize',12)
xlim([0 0.4])
grid on
box on

ei = 7.41; % Input Voltage as measured from battery
voltss = voltnew(end); % Steady State Voltage
K = abs(voltss)/ei; % Gain

R = 4; % ohm, measured 8 ohm from 2 motors
Td = interp1([6,12],[27,45],7.2); % oz-in, stall torque for 7.2V input
Kt = Td*R/6; % oz-in/A
Ke = Kt/141.6119; % (V/rad/s) 
B = ((Kt/K)-Ke*Kt)/R; % oz-in/rad/s
J = (tau*(R*B + Ke*Kt))/R; % oz-in-s^2

% PID Controller constants
Kp = 75; % 295.5; % P-Control
Ki = 35*40; % 5162; % I-Control
Kd = 1; % 2.951; % D-Control

Target = 10; % Distance from the wall to stop

%% Experimental data with load half speed
clear all;

nheaderlines = 31; 
dataexp = importdata('Motors on table with wires slower.lvm','\t',nheaderlines);

time = dataexp.data(:,1);
time = time(1837:2700);
time = time-time(1);
volt = dataexp.data(:,2);
volt = volt(1837:2700);
voltnew = volt;
timenew = time;

j = 1;
for i = 1:length(time)
    holder = volt(i);
    if (holder>(-6.99) || holder<(-7.32))
       indexes(j) = i;
       j = j+1;
    end
    
end
voltnew(indexes) = [];
timenew(indexes) = [];

voltnew(193) = [];
voltnew(276) = [];
voltnew(344) = [];

timenew(193) = [];
timenew(276) = [];
timenew(344) = [];

Vtau = voltnew(1)+0.632*(voltnew(end)-voltnew(1));
indexVtau = find(voltnew>Vtau-.004 & voltnew<Vtau+.004);
tau = mean(time(indexVtau)); 


figure
plot(timenew,voltnew,tau,Vtau,'ro')
xlabel('Time (sec)')
ylabel('Voltage (V)')
title('Output Voltage With Load at Half Speed')
legend('Experimental Data','Time Constant, \tau')
set(gca,'fontname','Times','fontsize',12)
xlim([0 timenew(end)])
grid on
box on

voltsmooth = smooth(voltnew);

figure
plot(timenew,voltsmooth,tau,Vtau,'ro')
xlabel('Time (sec)')
ylabel('Voltage (V)')
title('Output Voltage With Load at Half Speed')
legend('Smoothed Experimental Data','Time Constant, \tau')
set(gca,'fontname','Times','fontsize',12)
xlim([0 timenew(end)])
grid on
box on

ei = 7.41; % Input Voltage as measured from battery
voltss = voltnew(end); % Steady State Voltage
K = abs(voltss)/ei; % Gain

R = 4; % ohm, measured 8 ohm from 2 motors
Td = interp1([6,12],[27,45],7.2); % oz-in, stall torque for 7.2V input
Kt = Td*R/6; % oz-in/A
Ke = Kt/141.6119; % (V/rad/s) 
B = ((Kt/K)-Ke*Kt)/R; % oz-in/rad/s
J = (tau*(R*B + Ke*Kt))/R; % oz-in-s^2

%% Originall plot
clear all;

nheaderlines = 31; 
dataexp = importdata('Motors on table with wires slower.lvm','\t',nheaderlines);

figure;
hold on;
plot(dataexp.data(1800:2500,1),dataexp.data(1800:2500,2));
xlabel('Time (s)','FontSize',12);
ylabel('Voltage (V)','FontSize',12);
ylim([-8 -6]);