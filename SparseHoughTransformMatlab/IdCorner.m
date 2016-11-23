%Identify inner and outer corners 
% NumLine=20000;
% InnerCorFileName='zhLaserInCor.dat';
% %InnerCorFileName='oneline.dat';
% fid=fopen(InnerCorFileName);
% InPts=[];
% 
% % for cnt=1:NumLine
% %     InPts=[InPts, fscanf(fid ,'[%f,%f,%f]')];
% %     tline=fgets(fid);
% % end
% tline=1;
% 
% for cnt=1:20000
%     tline=fgets(fid);
% end
% cnt=0;
% while cnt<NumLine
%     cnt=cnt+1;
%     tmp=fscanf(fid ,'[%f,%f,%f]');
%     if isempty(tmp)
%         tline=fgets(fid);
%         tmp=fscanf(fid ,'[%f,%f,%f]');
%         if isempty(tmp)
%             break;
%         end
%     end
%     InPts=[InPts, tmp];
%     tline=fgets(fid);
% end
% fclose(fid);
% cnt
% 
% farest=70;
% NInPts = GetOffFarPts(InPts, farest);
% %NInPts=InPts;
% save('NInPts','NInPts');

load('NInPts');
plot3(NInPts(1,:),NInPts(2,:),NInPts(3,:),'o');
xlabel('x');
ylabel('y');
zlabel('z');
axis equal; 
axis([0 7 -8 8 -1 0]);
grid on;
title('Inner Corner Plot');
%print -depsc2 img/InnerPlot.eps


%%fit 3d inner corner
Area=[1.5 4 -1 0.5]';
%Height=[-1.5 -1]';
Height=[-1.27, 0.05]';
PFitThr=5e-3;
Corner=InnerCorner(NInPts,Area,Height, PFitThr)

CorRegBak=CorRegin;

plot3(CorRegin(1,:),CorRegin(2,:),CorRegin(3,:),'*');
axis equal;grid on;
xlabel('x');ylabel('y');zlabel('z');

hold on;
plot3(InnerCor(1,:),InnerCor(2,:),InnerCor(3,:),'ok');
plot3(InnerCor(1,CorIdx),InnerCor(2,CorIdx),InnerCor(3,CorIdx),'dg');

%idx=find(CorRegin(1,:)>3.4 & CorRegin(1,:)<3.6 & CorRegin(2,:)>-0.2 & CorRegin(2,:)<0 & CorRegin(3,:)<1.25)
idx=find(CorRegin(1,:)>3.4 & CorRegin(1,:)<3.8 & CorRegin(3,:)>-1.27)
CorRegin(3,idx)=-1.35;

idx=find(CorRegin(1,:)>0.4 & CorRegin(3,:)>-1.27)
CorRegin(3,idx)=-1.35;

hold on;
plot3(CorRegin(1,:),CorRegin(2,:),CorRegin(3,:),'*');

plot3(Corner(1),Corner(2),-1.5,'dr')

plot3(CorRegin(1,idx),CorRegin(2,idx),CorRegin(3,idx),'or');

CorLeft
plot3(CorLeft(1,:),CorLeft(2,:),CorLeft(3,:),'or');
plot3(CorRight(1,:),CorRight(2,:),CorRight(3,:),'ok');

print -deps2c img/Corner.eps
print -djpeg img/Corner.jpg


RErr=Rm*CorRight(1,:)+Rb-CorRight(2,:)