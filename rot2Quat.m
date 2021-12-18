% returns the quaternions [q0;q_vec that corresponds to the rotation matrix]
% Q = rot2Quat(R) takes in a rotation matrix and uses the
% equations described in class to change it to quaternion
% 
% Q = [4 1] matrix that has q0 in the first position and q1, q2, and q3 in
% the remaining rows respectively
% 
% R = Rotation matrix input
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function Q = rot2Quat(R)
q0 = sqrt((1+trace(R))/4);
if q0 == 0
    q1 = sqrt((1 + R(1,1) - R(2,2) - R(3,3)) / 4);
    q2 = sqrt((1 - R(1,1) + R(2,2) - R(3,3)) / 4);
    q3 = sqrt((1 - R(1,1) - R(2,2) + R(3,3)) / 4);
else
    q1 = (R(3,2) - R(2,3)) / (4 * q0);
    q2 = (R(1,3) - R(3,1)) / (4 * q0);
    q3 = (R(2,1) - R(1,2)) / (4 * q0);
end
Q = [q0,[q1, q2, q3]]';
end