% returns the rotation matrix encoded by a rotation of the theta radians
% about the unit vecotr k axis
% R = angleAxis2Rot(k, theta) takes in a rotation matrix and uses the
% equations described in class to change it to angle axis
% 
% R = Rotation matrix output
% 
% k = [3 1] vector that is the axis
% theta = angle in radians
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function R = angleAxis2Rot(k, theta)
I = eye(3);
k_x = [0, -k(3), k(2);
    k(3), 0, -k(1);
    -k(2), k(1), 0];
R = cos(theta)* I + (1-cos(theta))* k.*transpose(k) + sin(theta)*k_x;
end
