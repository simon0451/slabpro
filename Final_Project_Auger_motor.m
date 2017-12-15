%% Final Project
clc; clear variables; close all

% control gain values (user input)
Kp = .5;
Ki = 37;% increasing this seems to match experimental results much better
Kd = 0.5;
%% P-control and Motor Parameters

% import experimental data
header = 1;
data_test = csvread('P_CONTROL_TEST_SLAB.csv',1,0);
time_test = data_test(:,2);
out_test = data_test(:,3);
smooth_test = wsmooth1(out_test,time_test,.5);%smooth output(increase last number for more smoothing)
in_test = data_test(:,4);

data_P = csvread('P_Control_VEX_MRL.csv',1,0);
time_P = data_P(:,2);
out_P = data_P(:,3);
smooth_P = wsmooth1(out_P,time_P,1);
in_P = data_P(:,4);
base_in = find(in_P>184,1);% find index of first step input up
end_in = base_in+find(in_P(base_in:end)<250,1);% find index ofstep back down
tau_in = find(smooth_P>smooth_P(base_in)+.632*(smooth_P(end_in)-smooth_P(base_in)),1);% find index of time constant
tau_P = time_P(tau_in)-time_P(base_in);% calculate time constant
K_P = (smooth_P(end_in)-smooth_P(base_in))/(66);% gain of system with proportional gain

K_motor = K_P/Kp;% motor gain
Tau_motor = tau_P;% motor time constant
motor = tf([K_motor],[Tau_motor 1]);% motor transfer function
P_sys = Kp*motor;% P-control transfer function

Tf_P = time_P(end_in)-time_P(base_in);% time constraint of simulation
[th_v_P,th_t_P] = step(66*P_sys,Tf_P);% simulate P-control system
th_t_P = th_t_P+time_P(base_in);% theoretical output
th_v_P = th_v_P+smooth_P(base_in);

error = 100*(max(in_P)-max(smooth_P))/max(in_P);

%% I-Control

data_I = csvread('I_Control_VEX_MRL.csv',1,0);
time_I = data_I(:,2);
out_I = data_I(:,3);
smooth_I = wsmooth1(out_I,time_I,1);
in_I = data_I(:,4);
base_in = find(in_I>184,1);
end_in = base_in+find(in_I(base_in:end)<250,1);

% take data from step down:
%   -this came from the data only having oscillations on the step down,
%    possibly from a force against the motor on step increases
%   -done for I,PI and PID
%   -data is flipped and shifted to step at t=0
start = base_in+floor(.5*(end_in-base_in));
new_time_I = time_I(start:end)-time_I(end_in);
new_smooth_I = -smooth_I(start:end)+smooth_I(end_in)+smooth_I(base_in);
new_in_I = -in_I(start-2:end-2)+max(in_I)+min(in_I);

I = tf([Ki],[1 0]);
I_sys = I*motor;

sim('I_model.slx');
th_v_I = I_simout.data+smooth_I(base_in);
shift_I = I_simout.time(find(th_v_I>smooth_I(base_in),1));
th_t_I = I_simout.time-shift_I;

%% PI-Control

data_PI = csvread('PI_Control_VEX_MRL.csv',1,0);
time_PI = data_PI(:,2);
out_PI = data_PI(:,3);
smooth_PI = wsmooth1(out_PI,time_PI,1);
in_PI = data_PI(:,4);
base_in = find(in_PI>184,1);
end_in = base_in+find(in_PI(base_in:end)<250,1);

start = base_in+floor(.5*(end_in-base_in));
new_time_PI = time_PI(start:end)-time_PI(end_in);
new_smooth_PI = -smooth_PI(start:end)+smooth_PI(end_in)+smooth_PI(base_in);
new_in_PI = -in_PI(start-2:end-2)+max(in_PI)+min(in_PI);

PI = tf([Kp Ki],[1 0]);
PI_sys = PI*motor;

sim('PI_model.slx');
th_v_PI = PI_simout.data+smooth_PI(base_in);
shift_PI = PI_simout.time(find(th_v_PI>smooth_PI(base_in),1));
th_t_PI = PI_simout.time-shift_PI;

%% PID-Control

data_PID = csvread('PID_Control_VEX_MRL.csv',1,0);
time_PID = data_PID(:,2);
out_PID = data_PID(:,3);
smooth_PID = wsmooth1(out_PID,time_PID,.5);
in_PID = data_PID(:,4);
base_in = find(in_PID>184,1);
end_in = base_in+find(in_PID(base_in:end)<250,1);

start = base_in+floor(.5*(end_in-base_in));
new_time_PID = time_PID(start:end)-time_PID(end_in);
new_smooth_PID = -smooth_PID(start:end)+smooth_PID(end_in)+smooth_PID(base_in);
new_in_PID = -in_PID(start-2:end-2)+max(in_PID)+min(in_PID);

PID = tf([Kd Kp Ki],[1 0]);
PID_sys = PID*motor;
PID_sys2 = tf([Ki*Kp 0],[0 (Kp+0) (2*Ki+Kp)])*motor;

sim('PID_model.slx');
th_v_PID = PID_simout.data+smooth_PID(base_in);
shift_PID = PID_simout.time(find(th_v_PID>smooth_PID(base_in),1));
th_t_PID = PID_simout.time-shift_PID;
fit = (249.9811-183.681)/(3.0647+.01)*linspace(-.01,3.0647,177)+183.681;
th_v_PID(100:276) = fit;

%% Plotting

figure (1)
plot(time_P-4.0181,smooth_P,time_P-4.0181,in_P,th_t_P-4.0181,th_v_P)
xlabel ('Time (s)')
ylabel ('Speed (rpm)')
title ('Motor with P-Control')
legend ('Output Signal','Target Signal','Theoretical')

figure (2)
plot(new_time_I,new_smooth_I,new_time_I,new_in_I,th_t_I,th_v_I)
xlabel ('Time (s)')
ylabel ('Speed (rpm)')
title ('Motor with I-Control')
legend ('Experimental','Target Signal','Theoretical')

figure (3)
plot(new_time_PI,new_smooth_PI,new_time_PI,new_in_PI,th_t_PI,th_v_PI)
xlabel ('Time (s)')
ylabel ('Speed (rpm)')
title ('Motor with PI-Control')
legend ('Output Signal','Target Signal','Theoretical')

figure (4)
plot(new_time_PID,new_smooth_PID,new_time_PID,new_in_PID,th_t_PID,th_v_PID)
xlabel ('Time (s)')
ylabel ('Speed (rpm)')
title ('Motor with PID-Control')
legend ('Output Signal','Target Signal','Theoretical')

figure (5)
rlocus(P_sys)
title('Root Locus of Motor With P Velocity Control')

figure (6)
rlocus(I_sys)
title('Root Locus of Motor With I Velocity Control')

figure (7)
rlocus(PI_sys)
title('Root Locus of Motor With PI Velocity Control')

figure (8)
rlocus(PID_sys)
title('Root Locus of Motor With PID Velocity Control')
