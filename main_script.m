clear; close all; clc;
% Simulation parameters *** Edit these ***
N=60; % Number of seconds of robot movement to simulate
pps=500; % Resolution of simulation (points per second)

% Control parameters *** Edit these ***
spd = 0.2; % Initial speed of robot (m/s)
dT = 0.1; % Time interval between controller changes (s)
% Not - make sure your control interval is a multiple of your simulation frequency! Otherwise this won't work.
% E.g. Sim frequency = 1000Hz; Control frequency = 100Hz = check every 10 intervals of simulation.
ctrl_enable=4; % Enable control. 0 = no control; 1 = proportional control. Add cases as necessary!

% Robot parameters *** Edit these ***
robot.r = 0.035;    % wheel radius (m)
robot.wMax = 1.5;     % max motor angular velocity (rotations/second)
robot.d = 0.3;      % distance between wheels (m)

% Initalize simulation - No need to change this!
robot.setSpeed = spd; % Set desired speed m/s
T=linspace(0,N,pps*N); % Create vector of time points
robot.path = zeros(2,pps*N); % x,y position of robot at all simulated time points
robot.dir = zeros(2,pps*N); robot.dir(:,1)=[1 0]; % robot starts off pointing in the x direction
robot.lWheel=zeros(2,pps*N); robot.lWheel(:,1) = [0; -robot.d/2]; % track wheel positions L / R
robot.rWheel=zeros(2,pps*N); robot.rWheel(:,1) = [0; +robot.d/2];
robot.encL=zeros(1,pps*N); robot.encR=zeros(1,pps*N); % Simulate encoder readings

% Run simulation *** Implement your control schemes inside this function *** 
robot=drive_robot(robot,T,dT,ctrl_enable);

% Plot and analyse results *** Edit this section ***
figure;
hold on;
scatter(robot.path(1,:),robot.path(2,:),'b');
scatter(robot.lWheel(1,:),robot.lWheel(2,:),'k');
scatter(robot.rWheel(1,:),robot.rWheel(2,:),'k');
axis tight
plot(xlim, [0 0], '-r')
hold off;
title('Path travelled by robot');
xlabel('x (m)'); ylabel('y (m)');


