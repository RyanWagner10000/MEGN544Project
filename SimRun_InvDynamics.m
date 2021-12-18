clear; clc; close all;

%% Set Simulation Parameters
enable_control = true;
drawCSM = true;  % Draws CSM on the plot, requires a points3D.mat file to exist
Sim_Exact = false; % sets if the simulated geometry is exact (true) or slightly different (false)
simTime = 15; %Run the simulation for the specified number of seconds
makeMovie = false;
Laser_Feedback = true;
plot_arm = true; % controls if the arm is plotted during simulation. Slows down simulink a bit... 


%% verify necessary functions exist
assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('newtonEuler.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('constAccelInterp.m','file')==2,'Simulation Error:  Need to add project files to path');
assert(exist('createLink.m','file')==2,'Simulation Error:  Need to add project files to path');


%% Controller Parameter Definitions
run Geometry.m; % creates a linkList in the workspace for you and for the simulation
% load Trajectory_File.mat; % loads the CSM trajectory points


%% Define Kp and Kd gains
% Remember that the control update rate is 0.001 seconds
% Remember that the feedback linearization makes the system look like a
% pure integrator that is: A = [0 1; 0 0] and B = [0; 1]

% Kp = 300; % you need to update these in a meaningful way
% Kd = 100; % you need to update these in a meaningful way

% Kp = 200; % did worse
% Kd = 50; % got smaller and less accurate

% Kp = 400; % did better
% Kd = 200; % more accurate, but still not starting in the right place

% Kp = 600; % the C is dogshit, but it's closer to the actual location
% Kd = 500; % kind of writes crooked

% Kp = 600; % the C is A LOT better, starts closer
% Kd = 100; % M is now a little fucked up

% Kp = 800; % nothing really changed from the last one
% Kd = 100; % M is still a little fucked up, but ends at 0 angle config

% Kp = 400; % Draws CSM REALLY WELL, but slightly below target
% Kd = 50; % Top of S is little wonky, but rest fine

% Kp = 300; % Everything got a shit ton worse
% Kd = 75; % don't go above 75-100

% Kp = 450; % Draws CSM REALLY WELL, but still below target
% Kd = 50; % Changed gravity to 9.81 in newton_euler func, no change

% Kp = 450; % Draws CSM REALLY WELL, but still below target, top S wonky
% Kd = 50; % Changed gravity to 9.81 in Y dir in newton_euler func, no change

% Kp = 425; % really squiggley, not accurate
% Kd = 40; % more on target

% Kp = 430; % most on target yet, less squiggley, still not the best writing
% Kd = 40; % no gravity accounted in NE func: [0;0;0]

% Kp = 435; % writes neater, on target 99%
% Kd = 50; % still slightly below

% Kp = 435; % still on target, little behind letters though 
% Kd = 50; % pos gravity accounted in NE func: [0;9.8;0]

% Kp = 435; % right on the FUCKING MONEY, little shakey though
% Kd = 50; % pos gravity accounted in NE func: [0;0;9.8]

% Kp = 435; % C did better, S is still a little shakey
% Kd = 100; %

% Kp = 450; % C did worse, S is better
% Kd = 200; %

% Kp = 400; % everything is better, but still not 100%
% Kd = 150; % 

% Kp = 300; % everything is a lot better, but still not 100%
% Kd = 100; % S is still wonky

% Kp = 200; % C is more fucked up
% Kd = 100; % 

% Kp = 200; % nah, everything is bad
% Kd = 500; % 

% Kp = 100; % doesn't get the starting of the C
% Kd = 75; % 

% Kp = 500; % S is a little off, everything else is okay
% Kd = 150; % 

% Kp = 600; % C is a litle off but everything else perfect
% Kd = 200; % 

% Kp = 550; % C is a litle off but everything else perfect
% Kd = 175; % a little better

% Kp = 500; % C and S more wonky now
% Kd = 175; % 

% Kp = 550; % oh baby, that's the shit
% Kd = 125; % 

% Kp = 560; % better
% Kd = 125; % 

% Kp = 560; % same
% Kd = 115; % 

% Kp = 570; % little better
% Kd = 115; % 

% Kp = 580; % same
% Kd = 115; % 

% Kp = 580; % same
% Kd = 105; % 

% Kp = 600; % same
% Kd = 115; %

% Kp = 625; % less good
% Kd = 115; %

% Kp = 615; % same
% Kd = 125; %

% Kp = 675; % C = flawless, S and M good
% Kd = 125; %

Kp = 700; % oh yeah, that's  it
Kd = 125; %



%% Define Your Trajectory. I like to make a makeTrajectory function to help
% trajectory = makeTrajectory( simTime, ramp_time);   
% trajectory = [];
% trans_percent = .05;

load ('Trajectory_File.mat', 'path_length', 'points3D', 'transform_list');
trajectory = zeros(length(points3D)+2,8); % [time, x-pos, y-pos, z-pos, q0, q_vec[1], q_vec[2], q_vec[3]]
totalPathLength = sum(path_length);
drawTime = simTime - 2;

%Initial start, time to get to beginning of 'C'
trajectory(1,:) = [0,0,0,0,1,0,0,0];
% Q = normalize(rot2Quat(transform_list{1}(1:3,1:3)));
Q = rot2Quat(transform_list{1}(1:3,1:3));
if Q(3) ~=0 && Q(4) ~= 0
    if sign(Q(3)) == 1 && sign(Q(4)) == 1
        Q(4) = -1*Q(4);
    end
end
trajectory(2,:) = [1, points3D(1,1), points3D(1,2), points3D(1,3), Q(1), Q(2), Q(3), Q(4)];


for i = 2:1:length(path_length)
    timePercentage = (path_length(i-1)/totalPathLength)*drawTime;
    Q = rot2Quat(transform_list{i}(1:3,1:3));
%     Q = normalize(rot2Quat(transform_list{i}(1:3,1:3)));
    if Q(3) ~=0 && Q(4) ~= 0
        if sign(Q(3)) == 1 && sign(Q(4)) == 1
            Q(4) = -1*Q(4);
        end
    end
    trajectory(i+1,:) = [trajectory(i,1)+timePercentage, points3D(i,1), points3D(i,2), points3D(i,3), Q(1), Q(2), Q(3), Q(4)];
%     trajectory(i+1,:) = [trajectory(i,1)+timePercentage, transform_list{i}(1,4), transform_list{i}(2,4), transform_list{i}(3,4), Q(1), Q(2), Q(3), Q(4)];
end

%Finalize ending, time to get back to zero-angle
% Q = normalize(rot2Quat(eye(3)));
Q = rot2Quat(eye(3));
trajectory(end,:) = [simTime, 0.3740, 0.0, 0.63, 0, 0.7071, 0, 0.7071];

%% Controller Parameter Definitions
Sim_Name = 'System_InvDynamics';
%run Geometry.m; % creates a linkList in the workspace for you and for the simulation
if drawCSM
    sizeCSM = size(points3D,1);
else
    sizeCSM = 0;
end
open([Sim_Name '.slx']);


%% Enable/Disable Laser Feedback
set_param([Sim_Name '/Laser_Feedback'],'sw', int2str(Laser_Feedback));

%% Set Gains
set_param([Sim_Name '/Kp'],'Gain',['[' num2str(reshape(Kp,[1,numel(Kp)])) ']']);
set_param([Sim_Name '/Kd'],'Gain',['[' num2str(reshape(Kd,[1,numel(Kd)])) ']']);

%% Choose Simulation System (perfect model or realistic model)
set_param([Sim_Name '/ABB Arm Dynamics/sim_exact'], 'sw', int2str(Sim_Exact))

%% Enable/Disable Control
set_param([Sim_Name '/control_enable'], 'sw', int2str(enable_control))

%% Run Simulation
simOut =  sim(Sim_Name,'SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');


%% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');
pose_error = simOut.get('pose_error');

%% Plot theta as a function of time
figure(1)
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
saveas(gcf,'Theta Set','jpg')

figure(3)
subplot(2,1,1)
semilogy(pose_error.time,sqrt(sum(pose_error.signals.values(:,1:3).^2,2)));
title(['Position Pose Error']);
xlabel('time (s)');
ylabel('Norm Error (m)');
grid on;
subplot(2,1,2)
semilogy(pose_error.time,sqrt(sum(pose_error.signals.values(:,4:6).^2,2)));
title(['Angular Pose Error']);
xlabel('time (s)');
ylabel('Norm Error (rad)');
grid on;
saveas(gcf,'Pose Error','jpg')

figure(4)
subplot(2,1,1)
plot(control_torque.time,control_torque.signals.values);
title(['Control Torque Requested - Saturation Region']);
xlabel('time (s)');
ylabel('Torque [N-m]');
ylim([-350,350])
grid on;
subplot(2,1,2)
plot(control_torque.time,control_torque.signals.values);
title(['Control Torque Requested - Full Scale']);
xlabel('time (s)');
ylabel('Torque [N-m]');
grid on;
saveas(gcf,'Control Torque','jpg')


%% Display Arm Motion Movie

if makeMovie
    obj = VideoWriter('arm_motion','MPEG-4');
    obj.FrameRate = 30;
    obj.Quality = 50;
    obj.open();
end

figure(2)
plot(0,0); ah = gca; % just need a current axis handel
fh = gcf;
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
    plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
    hold on;
    plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
        reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
        reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
    if drawCSM
        plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
    end
    hold off;
    if makeMovie
        obj.writeVideo(getframe(fh));
    end
    pause(1/30)
end
if makeMovie
    obj.close();
end

saveas(gcf,'Trajectory','jpg')
