% returns the rotation matrix that corresponds to the quaternion
% R = quat2Rot(Q) takes in a quaternion matrix and uses the
% equations described in class to make a rotation matrix
% 
% k = [3 1] vector that is the axis
% theta = angle in radians
% 
% R = Rotation matrix output
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function R = quat2Rot(Q)
r11 = (Q(1)^2)+(Q(2)^2)-(Q(3)^2)-(Q(4)^2);
r12 = 2*Q(2)*Q(3)-2*Q(1)*Q(4);
r13 = 2*Q(1)*Q(3)+2*Q(2)*Q(4);
r21 = 2*Q(1)*Q(4)+2*Q(2)*Q(3);
r22 = (Q(1)^2)-(Q(2)^2)+(Q(3)^2)-(Q(4)^2);
r23 = 2*Q(3)*Q(4)-2*Q(1)*Q(2);
r31 = 2*Q(2)*Q(4)-2*Q(1)*Q(3);
r32 = 2*Q(1)*Q(2)+2*Q(3)*Q(4);
r33 = (Q(1)^2)-(Q(2)^2)-(Q(3)^2)+(Q(4)^2);
R = [r11, r12, r13;
    r21, r22, r23;
    r31, r32, r33];
% R = (Q(0) * Q(0) - Q(1)' * Q(1)) * eye(3) +...
%     2 * Q(0) * cpMap(Q(1)) + 2 * Q(1) * Q(1)';
end