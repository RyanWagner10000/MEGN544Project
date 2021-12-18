% roty computes the rotation matrix about the y axis
% R = rotY(theta)
% theta is passed into the function and placed into a matrix to calculate
% the y rotation
% R = Rotation matrix output
% 
% theta = angle in radians
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function R = rotY(theta)
R = [cos(theta), 0, sin(theta);
    0,1,0;
    -sin(theta), 0, cos(theta)];
end