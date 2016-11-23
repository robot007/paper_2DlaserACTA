function nearpts = GetOffFarPts(allpts, farest)
%allpts = FindNearPts(allpts, farest)
%don't draw points further than 'farest' memters
allpts=allpts';
[lineNum, dimension, ptNum] = size(allpts);

nearpts=[];
dist=zeros(1,ptNum);
for cnt1=1:lineNum
    tmp=sqrt( allpts(cnt1,1,:).^2 + allpts(cnt1,2,:).^2 + allpts(cnt1,3,:).^2 );
    dist(:) = tmp(1,1,:);
    idx= find( dist < farest & dist > 0.01 );
    nearpts_perline = zeros(3,length(idx));
    nearpts_perline(:,:) = allpts(cnt1,1:3,idx);
    nearpts= [nearpts nearpts_perline];
    end
end

return;