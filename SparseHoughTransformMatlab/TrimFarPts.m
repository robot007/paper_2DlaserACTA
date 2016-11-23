function RetPts=TrimFarPts(pts,org,threshold)
%function RetPts=TrimFarPts(pts,org,threshold)
%  Trims all points far than 'threshold' distance 
%  ORG is the origin position
%  PTS is a 3 by n matrix that stores [x,y,z] values
%  THRESHOLD  is the threshold for trimming data points
% 
% Zhen Song Nov 2002 
% 

[row,col]=size(pts);
RetPts=[];
for cnt=1:col
    if norm(pts(:,cnt)-org) <= threshold
        RetPts=[RetPts, pts(:,cnt)];
    end
end