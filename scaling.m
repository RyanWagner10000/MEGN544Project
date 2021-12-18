clear;
clc;

load('points2D.mat', 'points_all');

World_i = [-0.01;-0.3;0.48];
World_f = [0.14;-0.3;0.4];
endd = length(points_all);
P_i = points_all(1,:);
P_f = points_all(endd,:);
scale = (P_f(1) - P_i(1))/(World_f(1)-World_i(1));


allWorldPoints = zeros(length(points_all),3);
allWorldPoints(1,:) = World_i;
allWorldPoints(end,:) = World_f;
for i = 2:1:length(points_all)-1
    allWorldPoints(i,1) = World_f(1)-((points_all(endd,1)-points_all(i,1))/scale);
    allWorldPoints(i,2) = -0.3;
    allWorldPoints(i,3) = World_f(3)-((points_all(endd,2)-points_all(i,2))/scale);
end
points3D = allWorldPoints;
save('points3D.mat','points3D');

figure(1)
hold on
T = eye(4);
for i = 1:1:length(points_all)-1
    from = [allWorldPoints(i,1),allWorldPoints(i,3)];
    to = [allWorldPoints(i+1,1),allWorldPoints(i+1,3)];
    theta = angleBetweenTwoPoints(from,to);
    R_i = rotY(theta);
    quiver3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),...
        0.02*R_i(1,1), 0.02*R_i(1,2), 0.02*R_i(1,3), 'r', 'LineWidth', 1);
    quiver3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),...
        0.02*R_i(2,1), 0.02*R_i(2,2), 0.02*R_i(2,3), 'g', 'LineWidth', 1);
    quiver3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),...
        0.02*R_i(3,1), 0.02*R_i(3,2), 0.02*R_i(3,3), 'b', 'LineWidth', 1);
    plot3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),'k.');
    T(1:3,1:3) = R_i;
    T(1:3,4) = [allWorldPoints(i,1);
        allWorldPoints(i,2);
        allWorldPoints(i,3)];
    transforms(:,:,i) = T;
end
i = i + 1;
quiver3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),...
    0.02*R_i(1,1), 0.02*R_i(1,2), 0.02*R_i(1,3), 'r', 'LineWidth', 1);
quiver3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),...
    0.02*R_i(2,1), 0.02*R_i(2,2), 0.02*R_i(2,3), 'g', 'LineWidth', 1);
quiver3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),...
    0.02*R_i(3,1), 0.02*R_i(3,2), 0.02*R_i(3,3), 'b', 'LineWidth', 1);
plot3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),'k.');
T(1:3,1:3) = R_i;
T(1:3,4) = [allWorldPoints(i,1);
    allWorldPoints(i,2);
    allWorldPoints(i,3)];
transforms(:,:,i) = T;
save('trans.mat','transforms');
grid on
% axis equal
xlabel('x')
ylabel('y')
zlabel('z')
hold off


figure(2)
hold on
for i = 1:1:length(points_all)
    plot3(allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3),'k.');
    if i ~= 1
        pts = [allWorldPoints(i-1,1), allWorldPoints(i-1,2), allWorldPoints(i-1,3);
            allWorldPoints(i,1), allWorldPoints(i,2), allWorldPoints(i,3)];
        line(pts(:,1), pts(:,2), pts(:,3))
    end
end
xlabel('x')
ylabel('y')
zlabel('z')
hold off

function th = angleBetweenTwoPoints(from_, to_)
th = atan2((to_(2)-from_(2)),(to_(1)-from_(1)));
end
