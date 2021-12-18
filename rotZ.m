% rotZ computes the rotation matriz about the z axis
% R = rotZ(theta)
% theta is passed into the function and placed into a matrix to calculate
% the z rotation
% R = Rotation matrix output
% 
% theta = angle in radians
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function R = rotZ(theta)
R = [cos(theta), -sin(theta),0;
    sin(theta), cos(theta),0;
    0,0,1];
end