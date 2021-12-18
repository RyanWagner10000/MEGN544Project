% constAccelInterp provides the position, velocity, and acceleration at
% time t
% 
% [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
% This function takes the input arguements and provides the position, 
% velocity, and acceleration at time t for a trajectory interpolated using
% the constant acceleration approach. Each of these are length M vectors.
% 
% th1       = real scalar value or 8x1 vector if th_last is provided or not
% th2       = real scalar value or 8x1 vector if th_last is provided or not
% th3       = real scalar value or 8x1 vector if th_last is provided or not
% th4       = real scalar value or 8x1 vector if th_last is provided or not
% th5       = real scalar value or 8x1 vector if th_last is provided or not
% th6       = real scalar value or 8x1 vector if th_last is provided or not
% reachable = boolean, true or false if the transformation can be reached
% 
% T_des     = desired 4x4 homogeneous transform
% th_last   = 6x1 vector of previous thetas used for a specific location
% 
% Ryan Wagner
% 10821236
% MEGN 544
% October 15, 2021

function [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
a = [0;0.270;0.070;0;0;0];
d = [0.290;0;0;0.302;0.;0.072];
alpha = [-pi/2;0;-pi/2;pi/2;-pi/2;0];
if exist('th_last','var')==0
% Theta 1
d_05 = T_des(1:3,4) - (d(6)*T_des(1:3,1:3)*[0;0;1]);
theta1(1) = atan2(d_05(2,1),d_05(1,1));
theta1(2) = pi+atan2(d_05(2,1),d_05(1,1));
% Theta 3
rot_01 = rotZ(theta1(1))*rotX(alpha(1));
rot_01_2 = rotZ(theta1(2))*rotX(alpha(1));
d_01 = d(1)*[0;0;1];
rot_10 = rot_01';
rot_10_2 = rot_01_2';
d_14 = (rot_10)*(d_05-d_01);
d_14_2 = (rot_10_2)*(d_05-d_01);
phi = atan2(d(4),a(3));
l = sqrt((d(4)^2)+(a(3)^2));
si(1) = 2 * atan2(sqrt((2*a(2)*l)+(d_14(1,1)^2)+(d_14(2,1)^2)-(a(2)^2)-(l^2)),sqrt((2*a(2)*l)-(d_14(1,1)^2)-(d_14(2,1)^2)+(a(2)^2)+(l^2)));
si(2) = 2 * atan2(sqrt((2*a(2)*l)+(d_14_2(1,1)^2)+(d_14_2(2,1)^2)-(a(2)^2)-(l^2)),sqrt((2*a(2)*l)-(d_14_2(1,1)^2)-(d_14_2(2,1)^2)+(a(2)^2)+(l^2)));
theta3(1:2) = pi - si(1:2) - phi;
% Theta 2
alpha1(1) = atan2(d_14(2,1),d_14(1,1));
alpha1(2) = atan2(d_14_2(2,1),d_14_2(1,1));
gamma(1) = atan2((l*sin(theta3(1)+phi)),(a(2)+(l*cos(theta3(1)+phi))));
gamma(2) = atan2((l*sin(theta3(2)+phi)),(a(2)+(l*cos(theta3(2)+phi))));
theta2(1:2) = alpha1(1:2) - gamma(1:2) + pi/2;
% Theta 5
rot_03 = rot_01*rotZ(theta2(1))*rotX(0)*rotZ(theta3(1))*rotX(-pi/2);
rot_03_2 = rot_01_2*rotZ(theta2(2))*rotX(0)*rotZ(theta3(2))*rotX(-pi/2);
rot_36 = (rot_03')*T_des(1:3,1:3);
rot_36_2 = (rot_03_2')*T_des(1:3,1:3);
R1 = rot_36;
R2 = rot_36_2;
theta5(1) = atan2(sqrt((R1(3,2)^2)+(R1(3,1)^2)),R1(3,3));
theta5(2) = atan2(sqrt((R2(3,2)^2)+(R2(3,1)^2)),R2(3,3));
% Theta 4%6
if sin(theta5(1)) == 0 || sin(theta5(2)) == 0

    
        theta4(1:2)=0;
        theta6(1)=atan2(R1(2,1),cos(theta5(1))*R1(1,1));
        theta6(2)=atan2(R1(2,1),cos(theta5(2))*R1(1,1));
else
theta4(1) = atan2((-R1(2,3)/sin(theta5(1))),(-R1(1,3)/sin(theta5(1))));
theta4(2) = atan2((-R2(2,3)/sin(theta5(2))),(-R2(1,3)/sin(theta5(2))));
theta6(1) = atan2((-R1(3,2)/sin(theta5(1))),(R1(3,1)/sin(theta5(1))));
theta6(2) = atan2((-R2(3,2)/sin(theta5(2))),(R2(3,1)/sin(theta5(2))));
end
th1(1:4,1) = theta1(1);
    th1(5:8,1) = theta1(2);
    
    th3(1:2,1) = theta3(1);
    th3(3:4,1) = theta3(2);
    th3(5:6,1) = theta3(1);
    th3(7:8,1) = theta3(2);
    
    th2(1:2,1) = theta2(1);
    th2(3:4,1) = theta2(2);
    th2(5:6,1) = theta2(1);
    th2(7:8,1) = theta2(2);
    [th4, th5, th6] = deal(zeros(8,1));
    for i=1:8
        if(i/2==0)
            th5(i,1) = theta5(2);
            th4(i,1) = theta4(2);
            th6(i,1) = theta6(2);
        else
            th5(i,1) = theta5(1);
            th4(i,1) = theta4(1);
            th6(i,1) = theta6(1);
        end
    end
    if isreal(th1) && isreal(th2) && isreal(th3) && isreal(th4) && isreal(th5) && isreal(th6)
        reachable=1;
    else 
        reachable=0;
    end
%--------------------------------------------------------------------------
else
% Theta 1
d_05 = T_des(1:3,4) - (d(6)*T_des(1:3,1:3)*[0;0;1]);
theta1(1) = atan2(d_05(2,1),d_05(1,1));
theta1(2) = pi+atan2(d_05(2,1),d_05(1,1));
if theta1(1)<pi
    while theta1(1)<pi
    theta1(1) = theta1(1)+(2*pi);
    end
end
if theta1(2)<pi
    while theta1(2)<pi
    theta1(2) = theta1(2)+(2*pi);
    end
end
if theta1(1)>pi
    while theta1(1)>pi
    theta1(1) = theta1(1)-(2*pi);
    end
end
if theta1(2)>pi
    while theta1(2)>pi
    theta1(2) = theta1(2)-(2*pi);
    end
end

[~,I1] = min((th_last(1)-theta1).^2);
th1 = theta1(I1);
% Theta 3
rot_01 = rotZ(th1)*rotX(alpha(1));
d_01 = d(1)*[0;0;1];
rot_10 = rot_01';
d_14 = (rot_10)*(d_05-d_01);
phi = atan2(d(4),a(3));
l = sqrt((d(4)^2)+(a(3)^2));
count=0;
temp = sqrt((2*a(2)*l)-(d_14(1,1)^2)-(d_14(2,1)^2)+(a(2)^2)+(l^2));
if ~isreal(temp)
    count=1;
end
si = 2 * atan2(sqrt((2*a(2)*l)+(d_14(1,1)^2)+(d_14(2,1)^2)-(a(2)^2)-(l^2)),...
    real(sqrt((2*a(2)*l)-(d_14(1,1)^2)-(d_14(2,1)^2)+(a(2)^2)+(l^2))));
theta3(1) = pi - si - phi;
theta3(2) = pi + si - phi;
temp = [0,0,0,0];
if theta3(1) < pi
    while theta3(1) < pi
    theta3(1) = theta3(1)+(2*pi);
    end
    temp(1) = 1;
end
if theta3(2) < pi
    while theta3(2) < pi
    theta3(2) = theta3(2)+(2*pi);
    end
    temp(2) = 1;
end
if theta3(1) > pi
    while theta3(1) > pi
    theta3(1) = theta3(1)-(2*pi);
    end
    temp(3) = 1;
end
if theta3(2) > pi
    while theta3(2) > pi
    theta3(2) = theta3(2)-(2*pi);
    end
    temp(4) = 1;
end
[~,I3] = min((th_last(3)-theta3).^2);
th3 = theta3(I3);
if count > 0
    if sum(temp) == 4
        th3 = pi + si - phi;
    else
        th3 = pi - si - phi;
    end
end

alpha1 = atan2(d_14(2,1),d_14(1,1));
gamma = atan2((l*sin(th3+phi)),(a(2)+(l*cos(th3+phi))));
theta2 = alpha1 - gamma + pi/2;
if theta2<pi
    while theta2<pi
    theta2 = theta2+(2*pi);
    end
end
if theta2>pi
    while theta2>pi
    theta2 = theta2-(2*pi);
    end
end
[~,I2] = min((th_last(2)-theta2).^2);
th2 = theta2(I2);
% Theta 5
rot_03 = rot_01*rotZ(th2-(pi/2))*rotX(alpha(2))*rotZ(th3)*rotX(alpha(3));
rot_36 = (rot_03')*T_des(1:3,1:3);
R1 = rot_36;
theta5(1) = atan2(sqrt((R1(3,1)^2)+(R1(3,2)^2)),R1(3,3));
theta5(2) = atan2(-sqrt((R1(3,1)^2)+(R1(3,2)^2)),R1(3,3));
if theta5(1)<-pi
    while theta5(1)<-pi
    theta5(1) = theta5(1)+(2*pi);
    end
end
if theta5(2)<-pi
    while theta5(2)<-pi
    theta5(2) = theta5(2)+(2*pi);
    end
end
if theta5(1)>pi
    while theta5(1)>pi
    theta5(1) = theta5(1)-(2*pi);
    end
end
if theta5(2)>pi
    while theta5(2)>pi
    theta5(2) = theta5(2)-(2*pi);
    end
end
[~,I5] = min((th_last(5)-theta5).^2);
t5 = theta5(I5);

if round(sin(t5),2) == 0
    if round(cos(t5),2) == 1
    thetaa5(1) = t5;
    thetaa5(2) = t5+(2*pi);
    [~,I55] = min((th_last(5)-thetaa5).^2);
    th5 = thetaa5(I55);
    Q = [2 0 1;0 2 1;1 1 0]\[2*th_last(4);2*th_last(6);atan2(R1(2,1),R1(1,1))];
    thetaa4(1) = Q(1);
    thetaa4(2) = Q(1)-pi;
    [~,I44] = min((th_last(4)-thetaa4).^2);
    th4 = thetaa4(I44);
    thetaa6(1) = Q(2);
    thetaa6(2) = Q(2)-pi;
    [~,I66] = min((th_last(6)-thetaa6).^2);
    th6 = thetaa6(I66);
    else
    thetaa5(1) = t5;
    thetaa5(2) = -t5;
    [~,I55] = min((th_last(5)-thetaa5).^2);
    th5 = thetaa5(I55);
    P = [2 0 -1;0 2 1;-1 1 0]\[2*th_last(4);2*th_last(6);atan2(R1(2,1),-R1(1,1))];
    th4 = P(1);
    th6 = P(2);
    end
else
    th5 = t5;
    th4 = atan2((-R1(2,3)/sin(th5)),(-R1(1,3)/sin(th5)));
    th6 = atan2((-R1(3,2)/sin(th5)),(R1(3,1)/sin(th5)));
end
    if isreal(th1) && isreal(th2) && isreal(th3) && isreal(th4) && isreal(th5) && isreal(th6) && count==0
        reachable=1;
    else 
        reachable=0;
    end
end