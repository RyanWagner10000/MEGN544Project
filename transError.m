% transError returns a vector of position and angle error
% [error_vector] = transError(Td, Tc)
% This returns a 6x1 vector, where the first 3 elements are position error
% (desred - current), and the last three elements are an angle-axis
% representation of rotation error. Both expressed in the shared base frame
% 
% error_vector = 6x1 vector describing the error as expressed in the shared
%                base frame
% 
% Td           = the homogenious matrix describing the desired coordinate
%                pose in the reference frame
% Tc           = the homogenious matrix describing the current coordinate
%                frame in the reference frame
% 
% Ryan Wagner
% 10821236
% MEGN 544
% November 2, 2021
function [error_vector] = transError(Td, Tc)
error_vector = zeros(6,1);
error_vector(1:3,1) = Td(1:3,4) - Tc(1:3,4);
error_vector(4:6,1) = rotationError(Td(1:3,1:3),Tc(1:3,1:3));
end