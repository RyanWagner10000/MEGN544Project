% rotX computes the rotation matrix about the x axis
% R = rotX(theta)
% theta is passed into the function and placed into a matrix to calculate
% the x rotation
% R = Rotation matrix output
% 
% theta = angle in radians
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function R = rotX(theta)
R = [1,0,0;
    0,cos(theta), -sin(theta);
    0,sin(theta), cos(theta)];
end

