% Returns the homogenous transformation matrix corresponding to a 6 element
% twist vector
% H = twist2Transform(t) takes in a twist and uses the
% equations described in class to get the transformation matrix
% 
% H = [4 4] transformation matrix
% 
% t = twist input
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function H = twist2Transform(t)
eOmega = expm(cpMap(t(4:6,1)));
theta = sqrt(t(4,1) * t(4,1) + t(5,1) * t(5,1) + t(6,1) * t(6,1));
k = t(4:6,1) ./ theta;
H = eye(4);
H(1:3, 1:3) = eOmega;
d = ((eye(3) - eOmega) * (cpMap(k)) + theta * (k * k'))*t(1:3,1);
H(1:3,4) = d;
end