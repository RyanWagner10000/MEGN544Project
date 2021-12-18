% returns a rotation matrix corresponding to a poll, pitch, yaw encoded
% rotation
% R = rpy2Rot(roll, pitch, yaw) takes three angles and build the rotation
% matric based on an XYZ transformation
% 
% R = Rotation matrix output
% 
% roll = angle in radians
% pitch = angle in radians
% yaw = angle in radians
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function R = rpy2Rot(roll, pitch, yaw)
R = rotZ(yaw)*rotY(pitch)*rotX(roll);
end