% velocityJacobian returns the velocity jacobian
% [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
% This returns the velocity jacobian of the manipulator given an array of
% links created by the createLink function and the current joint variables
% 
% Jv            = the velocity jacobian
% JvDot         = the time derivative of the velocity jacobian
% 
% linkList      = the list of joint parameters created with createLink
% paramList     = the current theta or d values for the joints, according
%                 to the robot's encoders (an Nx1 array)
% paramRateList = the current theta_dot and d_dot values for the joints
%                 (an NX1 array))
% 
% Ryan Wagner
% 10821236
% MEGN 544
% November 2, 2021

function [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
H_0N = dhFwdKine(linkList, paramList);
Jv = zeros(6,length(paramList));
[zs, ds] = deal(zeros(3,length(paramList)+1));
Jv(1:3,1) = cross([0;0;1],H_0N(1:3,4));
zs(1:3,1) = [0;0;1];
ds(1:3,1) = [0;0;0];
if (linkList(1).isRotary == 1)
    Jv(1:3,1) = cross([0;0;1],H_0N(1:3,4));
    Jv(4:6,1) = [0;0;1];
elseif (linkList(1).isRotary == 0)
    Jv(1:3,1) = [0;0;1];
    Jv(4:6,1) = [0;0;0];
end
for i = 1:1:length(paramList)
    H_0i = dhFwdKine(linkList(1:i),paramList(1:i));
    zs(1:3,i+1) = H_0i(1:3,3);
    ds(1:3,i+1) = H_0i(1:3,4);
    if (i == length(paramList))
        H_0i = dhFwdKine(linkList(1:i),paramList(1:i));
        zs(1:3,i+1) = H_0i(1:3,3);
        ds(1:3,i+1) = H_0i(1:3,4);
        break;
    else
        if (linkList(i+1).isRotary == 1)
            Jv(1:3,i+1) = cross(H_0i(1:3,3),H_0N(1:3,4)-H_0i(1:3,4));
            Jv(4:6,i+1) = H_0i(1:3,3);
        elseif (linkList(i+1).isRotary == 0)
            Jv(1:3,i+1) = H_0i(1:3,3);
            Jv(4:6,i+1) = [0;0;0];
        end
    end
end

if ~exist('paramRateList','var') % If paramRateList not given
    JvDot = [];
    return;
end
JvDot = zeros(6,length(paramList));
[w, d_dot] = deal(zeros(3,length(paramList)+1));

for i = 2:1:length(paramList)+1
    if (linkList(i-1).isRotary == 1)
        w(1:3,i) = w(1:3,i-1) + paramRateList(i-1).*zs(1:3,i-1);
        d_dot(1:3,i) = d_dot(1:3,i-1) + cross(w(1:3,i),(ds(1:3,i) - ds(1:3,i-1)));
    elseif (linkList(i-1).isRotary == 0)
        w(1:3,i) = w(1:3,i-1);
        d_dot(1:3,i) = d_dot(1:3,i-1) + cross(w(1:3,i),(ds(1:3,i) - ds(1:3,i-1))) + paramRateList(i-1).*zs(1:3,i-1);
    end
end

for i = 1:1:length(paramList)
    if (linkList(i).isRotary == 1)        
        JvDot(1:3,i) = cross(cross(w(1:3,i),zs(1:3,i)),(H_0N(1:3,4)-ds(1:3,i)))...
            + cross(zs(1:3,i),(d_dot(1:3,end)-d_dot(1:3,i)));
        JvDot(4:6,i) = cross(w(1:3,i),zs(1:3,i));
    elseif (linkList(i).isRotary == 0)
        JvDot(1:3,i) = cross(w(1:3,i),zs(1:3,i));
        JvDot(4:6,i) = [0;0;0];
    end
end
end