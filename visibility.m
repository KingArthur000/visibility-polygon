N = 35;
[x, y] = meshgrid (1:N);
tri = delaunay (x(:), y(:));
%z = peaks (N);
z = readmatrix('pos.xlsx','Sheet','Sheet1','Range','B2:EL142');
zmod = zeros(35,35);
[m, n] = size(z);
for i = 1:m/4
 for j = 1:n/4
     zmod(i,j) = sum(sum(z(4*(i-1)+1:4*i,4*(j-1)+1:4*j)))/15;
 end
end
%trisurf (tri, x, y, z, "facecolor", "interp");
vertices = [x(:) y(:) zmod(:)];
vert1 = vertices(tri(:,1),:);
vert2 = vertices(tri(:,2),:);
vert3 = vertices(tri(:,3),:);
orig = [15 15 1];
colormap ("default");

q=1;
w=1;
endpts = zeros(10000,3);
endpt = zeros(10000,3);
theta = 0:pi/18:2*pi;
k = 100;%height

for rad = 20:10:50
    st = (rad*tan(pi/12)*sin(theta));
    ct = (rad*tan(pi/12)*cos(theta));
    for i =1:length(theta)
         figure(2);
         coord = [ct(i)+orig(1) st(i)+orig(2) k+orig(3)];
         dir = (coord-orig)./(norm(coord-orig));
         pt = orig+k*dir;
         [intersect ,t] = TriangleRayIntersection(orig, dir, vert1, vert2, vert3);
         %plot3(orig(1), orig(2), orig(3),orig(1) + 10*dir(1), orig(2) + 10*dir(2), orig(3) + 10*dir(3));
         line('XData',orig(1)+[0 k*dir(1)],'YData',orig(2)+[0 k*dir(2)],'ZData', orig(3)+[0 k*dir(3)],'Color','r','LineWidth',3)
         hold on;
         index = find(intersect);
         fprintf('Number of: faces=%i, points=%i, intresections=%i, direction=%i %i %i\n', size(tri,1), size(vertices,1), sum(intersect), dir);
         %pts = orig
         figure(1);
         trisurf(tri,x,y,zmod, 'FaceAlpha', 0.9)
         hold on;
         figure(3);
         if ~isempty(index)
             endpts(q,:) = [orig(1)+min(t(index(:)))*dir(1) orig(2)+min(t(index(:)))*dir(2) orig(3)+min(t(index(:)))*dir(3)];
             endpt(w,:) = [orig(1)+min(t(index(:)))*dir(1) orig(2)+min(t(index(:)))*dir(2) orig(3)+min(t(index(:)))*dir(3)];
             w = w+1;
             q = q+1;
             line('XData',orig(1)+[0 min(t(index(:)))*dir(1)],'YData',orig(2)+[0 min(t(index(:)))*dir(2)],'ZData', orig(3)+[0 min(t(index(:)))*dir(3)],'Color','r','LineWidth',3)
         end
         if isempty(index)
             line('XData',orig(1)+[0 k*dir(1)],'YData',orig(2)+[0 k*dir(2)],'ZData', orig(3)+[0 k*dir(3)],'Color','r','LineWidth',3)
             endpt(w,:) = orig + k*dir;
             w=w+1;
         end
        hold on;
    end
end
figure(2);
title (sprintf ("points of intersection on the ray"));
endind = find(endpts(:,1));
scatter3(endpts(endind,1),endpts(endind,2),endpts(endind,3),'b')

figure(5);
title (sprintf ("points of intersections"));
scatter3(endpts(endind,1),endpts(endind,2),endpts(endind,3),'b')

figure(6);
title (sprintf ("The visibiity polygon(1)"));
endind = find(endpt(:,1));
endpt = [endpt(endind,1) endpt(endind,2) endpt(endind,3)];
DT = delaunayTriangulation(endpt(:,1),endpt(:,2),endpt(:,3));
DT.Points(end+1,:) = orig;
tetramesh(DT,'FaceAlpha',0.3);
hold on;
[K,~] = convexHull(DT);
trisurf(K,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3),'FaceAlpha',0.1)
hold on;

figure(3);
title (sprintf ("The visibility polygon(2) with rays"));
DT = delaunayTriangulation(endpt(:,1),endpt(:,2),endpt(:,3));
DT.Points(end+1,:) = orig;
tetramesh(DT,'FaceAlpha',0.3);
hold on;
[K,~] = convexHull(DT);
trisurf(K,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3),[0 0 1])
hold on;

figure(4);
title (sprintf ("The visibility polygon(2)"));
endpt = [orig(1) orig(2) orig(3);endpt(endind,1) endpt(endind,2) endpt(endind,3);];
k = boundary(endpt);
hold on
trisurf(k,endpt(:,1),endpt(:,2),endpt(:,3),'Facecolor','red','FaceAlpha',0.1)

figure(1);
title (sprintf ("The visibility polygon(2) on the map"));
trisurf(k,endpt(:,1),endpt(:,2),endpt(:,3),'Facecolor','red','FaceAlpha',1)