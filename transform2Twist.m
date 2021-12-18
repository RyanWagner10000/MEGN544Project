% Returns the twist vector vector corresponding to the provided homogenous
% transform matrix.
% t = transform2Twist(H) takes in a transformation matrix and uses the
% equations described in class to get the twist form
% 
% t = stacked twist form [v;Omega]
% 
% H = Transformation matrix input
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function t = transform2Twist(H)
[k, theta] = rot2AngleAxis(H(1:3,1:3));
omega = k.*theta;
if theta == 0
    t = [H(1:3,4);[0,0,0]'];
else
    v = ((eye(3) - expm(cpMap(omega))) * cpMap(k) + omega * k')^-1 * H(1:3,4);
    t = [v;omega(1);omega(2);omega(3)];
end
end