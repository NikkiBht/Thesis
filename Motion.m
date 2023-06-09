% Show the motion of the manipulator
clc; clear all; close all;
global debug csv;    % Flag 0 or 1
global a tmin tmax;  % Geometry of the manipulator
global Ctrj Jtrj;    % Carteisan path and recoprded Joint trajectory

mode = 'r';          % Redundancy resolution for mid-joint
debug = 1; csv = 0;
a = [0.0 2.0 1.5 1.0;     % Home joint positions including EE
     0.0 0.0 0.0 0.0];
tmin=[  0.0;  0.0;  0.0]; % Minimum joint limits
tmax=[ pi/2; pi/2;   pi]; % Maximum joint limits

ts =[ pi/4; pi/4; pi/2];  % Home configuration of the manipulator
t=ts;

f1=figure;           % Show the manipulator motion
Show_Manip(t);       % Manipulator shown in red when out of limits

p0 = [-1; 2];        % Approach the first point with mode
t=MoveTo(p0,t,mode); % Move to the first point with mode
Ctrj(1,:)=p0';       % Initialize the first point of the path
Jtrj(1,:)=t';        % First joint position to record
csv=1;               % Start the recording

p1 = [ 1; 2];     
t=MoveTo(p1,t,mode); % Segment 1
p2 = [ 1; 4];
t=MoveTo(p2,t,mode); % Segment 2
p3 = [-1; 4];
t=MoveTo(p3,t,mode); % Segment 3
p4 = [-1; 2];
t=MoveTo(p4,t,mode); % Segment 4
csv=0; Jtrj;     % Stop recording

f2=figure              % Show the joint trajectory
plot(Jtrj(:,1),'-r');  % Jtrj could be save in a csv file.
hold on; grid on;
plot(Jtrj(:,2),'-g');
plot(Jtrj(:,3),'-b');
title('Square trajectory - Redundancy resolution');
xlabel('Simutation points');
ylabel('Joint position (radian)');
legend({'t1','t2','t3'},'Location','northeast');
axis([0 200  0 pi]);   % Joint workspace