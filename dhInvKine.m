% Returns the parameter list and residual error
% [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
% This returns the parameter list, according to the robot's encoders,
% necessary to achieve a desired homogeneous transform.
% 
% paramList      = list of parameters to encoders (6x1)
% error          = error in the residual transform
% 
% paramListGuess = an initial guess at the parameters, according to the
%                  robot's encoders
% desTransform   = the desired homogeneous transform
% linkList       = a list of the joint parameters created by createLink
% 
% Ryan Wagner
% 10821236
% MEGN 544
% November 2, 2021

function [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
paramList = paramListGuess;
for i = 1:10
    T = dhFwdKine(linkList,paramList);
    error = transError(desTransform,T);
    [Jv,~] = velocityJacobian(linkList,paramList);
    [U,S,V] = svd(Jv);
    Sinv = S';
    Sinv(Sinv/Sinv(1,1) > sqrt(eps)) = 1./Sinv(Sinv/Sinv(1,1) > sqrt(eps));
    Sinv(1./Sinv < sqrt(eps)) = 0;
    Jv_inv = (V)*Sinv*(U');
    
    diff = Jv_inv*error;
    paramList = paramList + diff;
    T = dhFwdKine(linkList,paramList);
    error = transError(desTransform,T);
    error = norm(error);
end
end