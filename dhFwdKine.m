% dhFwdKine will calculate the forward kinematics of a manipulator
% 
% H = dhFwdKine(linkList, paramList)
% This function takes the input arguements and finds the forward kinematics
% of a manipulator provided the inputs are in the form of DH parameter set
% 
% H             = transformation matrix from origin to end effector
% 
% linkList      = array of links created by createLink
% paramList     = array of current joint variables according to encoders
% 
% Ryan Wagner
% 10821236
% MEGN 544
% October 15, 2021


function H = dhFwdKine(linkList, paramList)
H = eye(4);
for i = 1:1:length(paramList)
    if (linkList(i).isRotary == 1)
        H = H * dhTransform(linkList(i).a, linkList(i).d, ...
            linkList(i).alpha, paramList(i)-linkList(i).offset);
    elseif (linkList(i).isRotary == 0)
        H = H * dhTransform(linkList(i).a, paramList(i)-linkList(i).offset, ...
            linkList(i).alpha, linkList(i).theta);
    else
        H = H * dhTransform(linkList(i).a, linkList(i).d, ...
            linkList(i).alpha, linkList(i).theta);
    end
end
end