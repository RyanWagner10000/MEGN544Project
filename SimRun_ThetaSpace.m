clear; clc; close all;

%% Set Simulation Parameters
enable_control = true;
drawCSM = true;  % Draws CSM on the plot, requires a points3D.mat file to exist
Velocity_Mode = true; % Enables theta_dot only based control
Sim_Exact = false; % sets if the simulated geometry is exact (true) or slightly different (false)
simTime = 16; %Run the simulation for the specified number of seconds
makeMovie = true;
uses_geometry = true;  %if true it makes a link-list for use in your functions

%% Define Theta space Trajectory
% [ time_end, th1->th6;...]

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
%% Load ABB Arm Geometry
if uses_geometry
    assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('newtonEuler.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('constAccelInterp.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('createLink.m','file')==2,'Simulation Error:  Need to add project files to path');
    run('Geometry.m'); 
end

if drawCSM 
    assert(exist('points3D.mat','file')==2,'Simulation Error: Need to make sure a file points3D.mat that has the transformed 2D points in it is on the path');
end 

if Velocity_Mode
    assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
end

%% Controller Parameter Definitions
Sim_Name = 'System_Theta_Space';
run Geometry.m; % creates a linkList in the workspace for you and for the simulation
if drawCSM
    load points3D; % loads the CSM trajectory points
    sizeCSM = size(points3D,1);
else
    sizeCSM = 0;
end
open([Sim_Name '.slx']);



%% Define Kp and Kd gains
% If yoru going to tweak, make sure you save these initial values.
Kp = [500;500;500;300;10;10];
Kd = [75;75;75;40;1;1];
Ki = [0;500;300;300;0;0]*0;
K = 15; %30


%% Get num points for simulink
traj_length = size(trajectory,1);

%% Choose Simulation System (perfect model or realistic model)
set_param([Sim_Name '/Theta Controller/ABB Arm Dynamics/sim_exact'], 'sw', int2str(Sim_Exact))

%% Enable/Disable Velocity Mode (Ignores Desired Theta Values)
set_param([Sim_Name '/Theta Controller/Velocity_Mode'], 'sw', int2str(~Velocity_Mode))


%% Enable/Disable Control
set_param([Sim_Name '/Theta Controller/control_enable'], 'sw', int2str(enable_control))


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

