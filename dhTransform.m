% Returns the homogenous transform corresponding to the provided DH
% parameters for a link
% H = dhTransform(a,d,alpha,theta) takes in four DH parameters and uses the
% equations described in class to get the transformation matrix
% 
% H = [4 4] transformation matrix
% 
% a = displacement in x direction
% d = displacement in z direction
% alpha = rotation around x axis
% theta = rotation around z axis
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function H = dhTransform(a,d,alpha,theta)
[TransZ, RotZ, TransX, RotX] = deal(eye(4));
TransZ(3,4) = d;
RotZ(1:3,1:3) = rotZ(theta);
TransX(1,4) = a;
RotX(1:3,1:3) = rotX(alpha);
H = TransZ * RotZ * TransX * RotX;
end