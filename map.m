%z = readmatrix('pos.xlsx','Sheet','Sheet1','Range','B2:EL142');
%N = 141;
%[x, y] = meshgrid (1:N);
%tri = delaunay (x(:), y(:));
%figure;
%trisurf(tri,x,y,z, 'FaceAlpha', 0.9);
x = gallery('uniformdata',[30 1],0);
y = gallery('uniformdata',[30 1],1);
z = gallery('uniformdata',[30 1],2);
DT = delaunayTriangulation(x(:),y(:),z(:));
figure;
tetramesh(DT,'FaceAlpha',0.3);
hold on;
[K,v] = convexHull(DT);
figure;
trisurf(K,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3),'Facecolor','red','FaceAlpha',0.1)
hold on;