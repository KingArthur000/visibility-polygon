% TEST:  Ray/box intersection using the Smits' algorithm
clc
X = [0;10;10;0;0];
Y = [0;0;10;10;0];
Z = [0;0;0;0;0];
figure;
plot3(X,Y,Z);   % draw a square in the xy plane with z = 0
plot3(X,Y,Z+10);
hold on;
grid on;
endpts = zeros(9261,3);
count = 1;

origin    = [5, 5, 0];
vmin      = rand(1,3)*5;  % vertex min
vmax      = vmin + 2;
for i = -1:0.1:1
for j = -1:0.1:1
for k = 0:0.1:1
direction = [i, j, k];
%origin    = [0, 4, 2];
%direction = [0.213, -0.436, 0.873];
%vmin      = [-1 2 1];        % vertex min
%vmax      = [ 3 3 3];        % vertex max

[flag ,tmin] = rayBoxIntersection(origin, direction, vmin, vmax);
intersection = [0 0 0];
if tmin>0
intersection = origin + tmin*direction;


    % box (voxel)
    vertices = [vmax(1) vmin(2) vmin(3); vmax(1) vmax(2) vmin(3); vmin(1) vmax(2) vmin(3); vmin(1) vmax(2) vmax(3); vmin(1) vmin(2) vmax(3); vmax(1) vmin(2) vmax(3); vmin; vmax ];
    faces = [1 2 3 7; 1 2 8 6; 1 6 5 7; 7 5 4 3; 2 8 4 3; 8 6 5 4];
    h= patch('Vertices',vertices,'Faces',faces,'FaceColor','green');
    %set(h,'FaceAlpha',0.5);
  
    % origin
    %text(origin(1), origin(2), origin(3), 'origin');
    %plot3(origin(1), origin(2), origin(3), 'k.', 'MarkerSize', 10);

    % direction
    %quiver3(origin(1), origin(2), origin(3), direction(1), direction(2), direction(3), 5);

    % intersection 
    %plot3(intersection(1), intersection(2), intersection(3), 'r.', 'MarkerSize', 15);
    x = [origin(1) intersection(1)];
    y = [origin(2) intersection(2)];
    z = [origin(3) intersection(3)];
    endpts(count,:) = intersection;
    count = count+1;
    plot3(x,y,z,'b')
end
    if norm(intersection) == 0
        intersection = 5*direction+origin;
        x = [origin(1) intersection(1)];
        y = [origin(2) intersection(2)];
        z = [origin(3) intersection(3)];
        endpts(count,:) = intersection;
        count = count+1;
        plot3(x,y,z,'b') 
    end
end
end
end

    length(endpts)
    k = boundary(endpts);
    hold on
    trisurf(k,endpts(:,1),endpts(:,2),endpts(:,3),'Facecolor','red','FaceAlpha',0.1)
    view(60,30);
    axis tight;
    xlabel('x');
    ylabel('y');
    zlabel('z');
