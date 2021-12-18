% Returns the roll, pitch, and yaw corresponding to a given rotation matrix
% [roll, pitch, yaw] = rot2RPY(R) takes in a rotation matrix and uses the
% equations described in class to get the angles in XYZ configuration
% 
% roll = angle in radians for the x direction
% pitch = angle in radians for the y direction
% yaw = angle in radians for the z direction
% 
% R = Rotation matrix output
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function [roll, pitch, yaw] = rot2RPY(R)
    roll = [atan2(R(3,2)/sqrt(R(3,2)^2+R(3,3)^2), R(3,3)/sqrt(R(3,2)^2+R(3,3)^2));
        atan2(R(3,2)/-sqrt(R(3,2)^2+R(3,3)^2), R(3,3)/-sqrt(R(3,2)^2+R(3,3)^2))];
    
    pitch = [atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
        atan2(-R(3,1),-sqrt(R(3,2)^2+R(3,3)^2))];
    
    yaw = [atan2(R(2,1)/sqrt(R(3,2)^2+R(3,3)^2),R(1,1)/sqrt(R(3,2)^2+R(3,3)^2));
        atan2(R(2,1)/-sqrt(R(3,2)^2+R(3,3)^2),R(1,1)/-sqrt(R(3,2)^2+R(3,3)^2))];
end