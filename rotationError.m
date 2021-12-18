% rotationError returns an angle-axis vector describing rotation
% rot_error_vector = rotationError(Rot_desired,Rot_current)
% This returns an angle-axis vector, expressed in the reference frame,
% describing what rotation is necessary to take Rot_current to Rot_desired
% 
% rot_error_vector = 3x1 vector describing the axis of rotation multiplied
%                    by and angle of rotation (rad) necessary to transform
%                    Rot_current into Rot_desired
% 
% Rot_desired      = rotation matrix describing the desired coordinate
%                    frame in the reference frame
% Rot_current      = rotation matrix describing the current coordinate
%                    frame in the reference frame
% 
% Ryan Wagner
% 10821236
% MEGN 544
% November 2, 2021
function rot_error_vector = rotationError(Rot_desired,Rot_current)
[k,theta] = rot2AngleAxis(Rot_desired*transpose(Rot_current));
rot_error_vector = k .* theta;
end