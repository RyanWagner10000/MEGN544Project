% Newton Euler computes the inverse dynamics
% 
% H = dhFwdKine(linkList, paramList)
% This function takes the input arguements and computes the inverse
% dynamics of a serial link manipulator
% 
% jointTorques          = torques of each joint in the linkage (Nx1)
% Jv                    = velocity Jacobian (6xN)
% JvDot                 = acceleration Jacobian (6xN)
% 
% linkList              = a list of the joint parameters created by
%                         createLink
% paramList             = the current joint angles/distances (Nx1)
% paramListDot          = the current joint angles/distance speeds (Nx1)
% paramListDDot         = the current joint angles/distances accelerations 
%                         (Nx1)
% boundry_conditions    = a structure containing:
%                         base_angular_velocity, 
%                         base_angular_acceleration,
%                         base_linear_acceleration (with gravity added in),
%                         distal_force,
%                         distal_torque
% 
% Ryan Wagner
% 10821236
% MEGN 544
% November 22, 2021

function [jointTorques, Jv, JvDot] = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundry_conditions)

% Number of Joints beyond base.
numJoints = length(linkList);

% Preallocate list of values stored in memory between loops for speed:
list = repmat(struct( 'zlast', zeros(3,1),...	% 0z_i-1, rotation axis for link i in base frame
                      'woi', zeros(3,1),...     % Angular velocity of origin i in base frame
                      'doi', zeros(3,1),...     % Position of Origin i relative to base in base frame
                      'doi_dot', zeros(3,1),...	% Velocity of Origin i relative to base in base frame
                      'Hi', eye(4),...          % Transformation matrix from Origin i-1 to i in i-1 frame
                      'Fi', zeros(3,1),...      % Inertial Force on link i in base frame
                      'Ni', zeros(3,1) ),...	% Inertial Torque on link i in base frame
                numJoints, 1 );

% Initialize link variables that get propagated forward:
Toi = eye(4); % Transform from 0 to joint i
v = zeros(3,1); % Velocity in base frame
w_base = boundry_conditions.base_angular_velocity;

W = w_base; % Angluar Velocity in joint i frame
Wdot = boundry_conditions.base_angular_acceleration; % Angular Acceleration in joint i frame
vdot = boundry_conditions.base_linear_acceleration; % Linear acceleration in joint i frame

% Forward iteration from bass frame to tool for kinematics:
for i = 1 : numJoints

    % Calculate link transform from i-1 to i in i-1 frame:
    link = linkList(i);
    H = dhFwdKine(link, paramList(i));

    % Important parameters for calculations:
    z = Toi(1:3,3); % z-axis orientation from base to joint i-1
    dprev = Toi(1:3,4); % Distance from base 0 to i-1 joint
    Toi = Toi*H; % Update base to link transform
    d0_sub = Toi(1:3,4) - dprev; % i-1 to i joint distance in base frame

    % CoM distance in base (0th) frame. NOTE: link.com = irii, from
    % origin i to the link i's center of mass.
    r0_sub = Toi(1:3,1:3)*(H(1:3,1:3)'*H(1:3,4) + link.com);

    % Update base frame velocity, acceleration, angular acceleration,
    % and angular velocity. Order matters!
    if link.isRotary

        % Joint i angular acceleration and velocity in base frame:
        Wdot = Wdot + paramListDDot(i)*z + paramListDot(i)*cross(W, z);
        W = W + paramListDot(i)*z;

        % CoM linear acceleration from i-1 to i, in base frame:
        vcdot = vdot + cross(Wdot, r0_sub) + cross( W, cross(W, r0_sub) );

        % Joint i linear acceleration and velocity in base frame:
        vdot = vdot + cross(Wdot, d0_sub) + cross( W, cross(W, d0_sub) );
        v = v + cross(W, d0_sub);

    else % It is prismatic, where angular motion remains constant:

        % CoM linear acceleration from joint i-1 to i, in base frame:
        vcdot = vdot + cross(Wdot, r0_sub) + cross( W, cross(W, r0_sub) ) + ...
            paramListDDot(i)*z + 2*paramListDot(i)*cross(W, z);

        % Joint i linear acceleration and velocity in base frame:
        vdot = vdot + cross(Wdot, d0_sub) + cross( W, cross(W, d0_sub) ) + ...
            paramListDDot(i)*z + 2*paramListDot(i)*cross(W, z);
        v = v + cross(W, d0_sub) + paramListDot(i)*z;

    end

    % Calculate and save Inertial Force and Torque in i'th frame:
    Rt = Toi(1:3,1:3)';
    ivcdot = Rt*vcdot;
    iw = Rt*W;
    iwdot = Rt*Wdot;
    f = link.mass*(ivcdot); % Newton's Equation
    n = link.inertia*(iwdot) + cross(iw, link.inertia*iw); % Euler's Equation

    % Save values specific to calculating Jv and JvDot:
    list(i).doi = Toi(1:3,4); % save 0_d_0i from base to joint i
    list(i).woi = W - w_base; % Save 0_w_0i without B.C.s
    list(i).doi_dot = v - cross(w_base, list(i).doi); % Save 0_v_0i without B.C.s
    list(i).zlast = z; % Save 0_z_i-1 vector

    % Save values specific to calculating joint forces / torques:
    list(i).Hi = H; % i-1_T_i for frame conversions
    list(i).Fi = f; % Inertial Force in i'th frame
    list(i).Ni = n; % Inertial Torque in i'th frame

end % End forward iterations

% Initialize variables for calculating Jv and JvDot:
Jv = zeros(6, numJoints); % preallocate Jv for speed
JvDot = zeros(6, numJoints); % preallocate JvDot for speed
doN = list(end).doi; % Extract Distance from Base to Tool in base frame
voN = list(end).doi_dot; % Extract Velocity of Tool in base frame

% Initialize and preallocate variables for force/torque propagation:
F = boundry_conditions.distal_force; % 3-D joint force
N = boundry_conditions.distal_torque; % 3-D joint torque
jointTorques = zeros(numJoints, 1); % Z-component of joint forces/torques

% Backwards iteration from last joint to base for kinetics and
% jacobian parts:
for i = numJoints:-1:1 % From last joint to base

    % Extract rotation matrix to convert i+1 -> i frame vectors:
    if i == numJoints
        Rii1 = eye(3); % No rotation occurs beyond distal end
    else
        Rii1 = list(i+1).Hi(1:3,1:3); % i_R_i+1 rotation is applied
    end
    R = list(i).Hi(1:3,1:3); % i-1_R_i rotation from i-1 to i frame

    % Update Force on joint i in i'th frame:
    F = list(i).Fi + Rii1*F;

    % Update Torque on joint i in i'th frame:
    N = list(i).Ni + Rii1*N + cross( R'*list(i).Hi(1:3,4), F ) + ...
        cross(linkList(i).com, list(i).Fi);

    % Displacement and velocity differences (without boundary
    % conditions) from frame i -> numJoints in base frame:
    if i > 1
        diN = doN - list(i-1).doi;
        viN = voN - list(i-1).doi_dot;
    else
        diN = doN;
        viN = voN;
    end

    % The following if-else conditions does two parts:
    %   > Set joint force or torque in i-1 frame by Z-component.
    %   > Set each column of the jacobian by joint type (end to base).
    %       NOTE: Our jacobians have no boundary conditions included.

    if linkList(i).isRotary % Rotational Joint

        % Joint i torque is the Z component in i-1 frame:
        jointTorques(i,1) = dot( [0;0;1], R*N ); 

        % Populate Jv and JvDot: 
        Jv(1:3,i) = cross( list(i).zlast, diN );
        Jv(4:6,i) = list(i).zlast;

        JvDot(1:3,i) = cross( cross(list(i).woi, list(i).zlast), diN ) + ...
            cross( list(i).zlast, viN );
        JvDot(4:6,i) = cross( list(i).woi, list(i).zlast );

    else % Prismatic

        % Joint i force is the Z component in i-1 frame:
        jointTorques(i,1) = dot( [0;0;1], R*F ); 

        % Populate Jv:
        Jv(1:3,i) = list(i).zlast;
        Jv(4:6,i) = zeros(3,1);

        % Populate JvDot:
        JvDot(1:3,i) = cross( list(i).woi, list(i).zlast );
        JvDot(4:6,i) = zeros(3,1);
    end
end
end