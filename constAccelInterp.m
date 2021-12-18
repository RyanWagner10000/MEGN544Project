% constAccelInterp provides the position, velocity, and acceleration at
% time t
% 
% function [p, v, a] = constAccelInterp(t, trajectory, transPercent))
% This function takes the input arguements and provides the position, 
% velocity, and acceleration at time t for a trajectory interpolated using
% the constant acceleration approach. Each of these are length M vectors.
% 
% p             = position vector
% v             = velocity vector
% a             = acceleration vector
% 
% t             = the time of interest
% trajectory    = a Nx(M+1) array of points. Col1 = time for each point in
%   trajectory, remaining M columns are points to be reached at that time
% transPercent  = the percentage of the trajectory time to use for the
%   constant acceleration transition. Range [0,0.5]
% 
% Ryan Wagner
% 10821236
% MEGN 544
% October 15, 2021

function [p, v, a] = constAccelInterp(t, trajectory, transPercent)
[p,v,a] = deal(zeros(1,2));
t1 = trajectory(:,1);
[m,n] = size(trajectory);

tau = zeros(1,m-1);
for j = 1:1:m-1
    tau(j) = transPercent*(t1(j+1)-t1(j));
end
tau(m) = tau(m-1);

pos = zeros(m,n);
for i = 2:1:n
    pos(:,i) = trajectory(:,i);
    for j = 2:1:m-1
        dp1 = pos(j)-pos(j-1);
        dp2 = pos(j+1)-pos(j);
        if j-1 == 1 && t > t1(j-1) && t < (t1(j-1)+tau(j-1))
            dp = pos(j)-pos(j-1);
            p(i-1) = pos(j-1,i)+(((t-t1(j-1)+tau(j-1))^2)*dp/(4*tau(j-1)*(t1(j)-t1(j-1))));
            a(i-1) = (1/(2*tau(j)))*(dp/(t1(j+1)-t1(j)));
            v(i-1) = (1/(2*tau(j)))*(dp*(t-t1(j)+tau(j))/(t1(j+1)-t1(j)));
            break;
        elseif j+1 == m && t < t1(j+1) && t > (t1(j+1)-tau(j+1))
            dp = pos(j+1,i)-pos(j,i);
            p(i-1) = pos(j+1,i)+(((t-t1(j+1)-tau(j+1))^2)*dp/(4*tau(j+1)*(t1(j+1)-t1(j))));
            a(i-1) = (1/(2*tau(j+1)))*(dp/(t1(j+1)-t1(j)));
            v(i-1) = (1/(2*tau(j+1)))*(dp*(t-t1(j+1)-tau(j+1))/(t1(j+1)-t1(j)));
            break;
        elseif t < (t1(j)-tau(j)) && t >= (t1(j-1)+tau(j-1))
            dp1 = pos(j,i)-pos(j-1,i);
            p(i-1) = pos(j,i)-(((t1(j)-t)/(t1(j)-t1(j-1)))*dp1);
            v(i-1) = dp1/(t1(j)-t1(j-1));
            a(i-1) = 0;
            break;
        elseif t > (t1(j)+tau(j)) && t <= (t1(j+1)-tau(j+1))
            dp2 = pos(j+1,i)-pos(j,i);
            p(i-1) = pos(j,i)+(((-t1(j)+t)/(t1(j+1)-t1(j)))*dp2);
            v(i-1) = dp2/(t1(j+1)-t1(j));
            a(i-1) = 0;
            break;
        elseif t <= (t1(j)+tau(j)) && t >= (t1(j)-tau(j))
            dp1 = pos(j,i)-pos(j-1,i);
            dp2 = pos(j+1,i)-pos(j,i);
            p(i-1) = pos(j,i)-(((t-t1(j)-tau(j))^2)*dp1/(4*tau(j)*(t1(j)-t1(j-1))))+(((t-t1(j)+tau(j))^2)*dp2/(4*tau(j)*(t1(j+1)-t1(j))));
            v(i-1) = ((dp2*(t-t1(j)+tau(j)))/(2*tau(j)*(t1(j+1)-t1(j))))-((dp1*(t-t1(j)-tau(j)))/(2*tau(j)*(t1(j)-t1(j-1))));
            a(i-1) = ((dp2)/(2*tau(j)*(t1(j+1)-t1(j))))-((dp1)/(2*tau(j)*(t1(j)-t1(j-1))));
            break;
        end
    end
end
end