function generateTransPointsMat(init_calc, points,step)
if (init_calc == true) %IF GETTING THE POINTS AND TRANSORMS INITIALLY
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

    T = eye(4);
    for i = 1:1:length(points_all)-1
        from = [allWorldPoints(i,1),allWorldPoints(i,3)];
        to = [allWorldPoints(i+1,1),allWorldPoints(i+1,3)];
        theta = angleBetweenTwoPoints(from,to);
        R_i = rotY(theta);
        T(1:3,1:3) = R_i;
        T(1:3,4) = [allWorldPoints(i,1);
            allWorldPoints(i,2);
            allWorldPoints(i,3)];
        transforms(:,:,i) = T;
    end
    i = i + 1;
    T(1:3,1:3) = R_i;
    T(1:3,4) = [allWorldPoints(i,1);
        allWorldPoints(i,2);
        allWorldPoints(i,3)];
    transforms(:,:,i) = T;
    save('transforms.mat','transforms');
    %----------------------------------------------------------------------
else
    T = eye(4);
    for i = 1:1:length(points)-1
        from = [points(i,2),points(i,3)];
        to = [points(i+1,2),points(i+1,3)];
        theta = angleBetweenTwoPoints(from,to);
        R_i = rotY(theta);
        T(1:3,1:3) = R_i;
        T(1:3,4) = [points(i,2);
            -0.3;
            points(i,3)];
        more_transforms(:,:,i) = T;
    end
    i = i + 1;
    T(1:3,1:3) = R_i;
    T(1:3,4) = [points(i,2);
        -0.3;
        points(i,3)];
    more_transforms(:,:,i) = T;
    save('more_transforms.mat','more_transforms');
end
end

function th = angleBetweenTwoPoints(from_, to_)
th = atan2((to_(2)-from_(2)),(to_(1)-from_(1)));
end