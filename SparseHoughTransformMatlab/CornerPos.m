%function CorPos=CornerPos(filestr)

%function CorPos=CornerPos(filestr)
% read 2D laser data form a file in filestr
% then find Corner's Position 
% the corner could either convex or concave
%
% Zhen Song Nov 2002

filestr='zhenLaserOutCor6.dat';
pts=ScanDatFile(filestr);
size(pts)
pts=TrimFarPts(pts,[0;0;0],50);
plot3(pts(1,:),pts(2,:),pts(3,:),'o');
xlabel('x');ylabel('y');zlabel('z');