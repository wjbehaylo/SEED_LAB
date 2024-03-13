% run_steeringsimulation
%
% runs a simulation of a two wheeled robot with a forward motion and anglual
% motion controller. This script sets up
% the parameters of the robot, and the input voltages as timeseries
% The simulink model 'steeringsimulation' is run, and the results plotted


% conversions from angle to counts and back
%%rad_to_counts = 3600/(2*pi);
%%counts_to_rad = 1/rad_to_counts;
wheelCircumference = pi * 0.475722; % measured wheel radius;
rotationCircumference = 1 * pi; % measured robot width;
%r_actual_ft = .5; % actual wheel radius
%b_actual_ft = 1; % actual robot width
Ts=.01; % sample time in seconds
%
% right wheel parameters
%
%K_r=1.5;
%sigma_r=15;
%K_r=1.0;
%sigma_r=10;
%
% left wheel parameters
%
%K_l=1;
%sigma_l=10;

%%%% variables %%%%
posKp = 3;
posKi = 0.33; 
velKp = 8;

%%% K0 is left wheel
K0 = 1.5; % DC gain (V/tick) 
sig0 = 15; % time constant reciprocal (1/s)

%%% K1 is right wheel
K1 = 1.0; % DC gain (V/tick)
sig1 = 10; % time constant reciprocal (1/s)

desiredAngle=timeseries([0 0],[0 10]); %%% phi_d == desiredAngle
desiredPosition=timeseries([0 0 2 2],[0 4.9 5 10]); %%% rho_d == desiredPosition

actualAngle = (rotationCircumference / out.actualPositionDiff) * 360;
actualPosition = out.actualPositionBase;

%%%% simulation %%%%%
out = sim('apctrlsystem.slx')
figure(1)
clf
plot(desiredAngle)
hold on
plot(desiredPosition)
plot(actualAngle)
plot(actualPosition)
set(gca,'fontsize',14)
legend('desired angle','desired position','actual angel','actual position','location','northwest')
xlabel('time (s)')
ylabel('magnitude')
title('Wheel Position')
%% 
figure(2)
clf
plot(out.simulation)
set(gca,'fontsize',14)
xlabel('Time (s)')
ylabel('Position (ft)')
legend('X','Y','Phi')
title('Robot Position')
figure(3)
animate
