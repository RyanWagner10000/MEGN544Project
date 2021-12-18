% returns the matrix packing of the cross product operator
% X = cpMap(w) takes in a vector and uses the
% equations described in class to build the cross product of it
% 
% w = vector input
% 
% X = matrix output
% 
% Ryan Wagner
% 10821236
% MEGN 544
% September 17, 2021
function X = cpMap(w)
X = [0, -w(3), w(2);
    w(3), 0, -w(1);
    -w(2), w(1), 0];
end