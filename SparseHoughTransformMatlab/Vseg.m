% segment by V
%

fid=fopen(filestr);
tline=fgets(fid);% first line: Data from laser
tline=fgets(fid);% second line: blank 
pts=[];
%while tline~=-1
Len=361;
for cnt=1:Len
    pts=[pts, fscanf(fid,'[%f,%f,%f]')];
    tline=fgets(fid);
end
fclose(fid);
plot3(pts(1,:),pts(2,:),pts(3,:),'o');


% origin 
%p0=[0; 0; 0];
%pts=pts-p0*ones(1,Len);
% Cpt=[pts(1,:); 0; pts(3,:)];
% d=sqrt(pts(1,:).^2+pts(2,:).^2+pts(3,:).^2);
% beta=atan2(pts(2,:), pts(1,:)); % 


v=pts(:,1:Len)-pts(