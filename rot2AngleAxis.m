% returns the angle and axis corresponding to a rotation matrix
% [k, theta] = rot2AngleAxis(R) takes in a rotation matrix and uses the
% equations described in class to change it to angle axis
% 
% k = [3 1] vector that is the axis
% theta = angle in radians
% 
% R = Rotation matrix input
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function [k, theta] = rot2AngleAxis(R)
magnitude = norm([R(3,2) - R(2,3), R(1,3) - R(3,1), R(2,1) - R(1,2)]);
theta = atan2(0.5*magnitude,(trace(R)-1)/2);
if theta == 0
    k = [1;0;0];
elseif (mod(theta,pi) == 0)
    temp = sqrt((R(1,1)+1)/2);
    k = [temp;(R(1,2)+R(2,1))/(4*temp);(R(1,3)+R(3,1))/(4*temp)];
else
    k = (1 / (2 * sin(theta))) .* [R(3,2) - R(2,3), R(1,3) - R(3,1), R(2,1) - R(1,2)]';
end
end