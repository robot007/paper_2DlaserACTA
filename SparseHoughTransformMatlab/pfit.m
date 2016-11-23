function [para, meanerr,fitptidx,varargout] = pfit(fitpts,thr,varargin)
%function [para, meanerr,fitptidx,varargout] = pfit(fitpts,thr,varargin)
%PFIT Plane fitting 
% Fit the plane for a maximum of 5 times, or until
% the error threshold meet.
%
%PTS points for plane fitting
%PARA parameters of the plane
%MEANERR mean error of the fitting
%FITPTIDX  the index of the fitted points
%THR threashold. If it is set, pick out wild values and fit again
%varargin 
%
% Nov ,02 : add median filter 
% Aug 23, 2001 
% Zhen Song, 
%


no_of_inputs = nargin;


[row col]=size(fitpts);
if col~=3 | row < 4
   disp('input points must be n by 3 matrix, where n>=4');
   return;
end
[U, S, V]=svd([fitpts, ones(row,1)],0);
para=V(:,4)';
if (no_of_inputs==1)
    TotalErr=sum( abs( [fitpts, ones(row,1)] * para'), 1);
    meanerr=TotalErr/row;
    return;
else %3 inputs
    err=[fitpts,ones(row,1)]*para'; %x*p1+y*p2+z*p3+p4=0
%    [allrow,tmp]=size(allpts);
%    allErr=[allpts,ones(allrow,1)]*para';
    meanerr=mean(abs(err));
    MaxLoop=5;%do not run the loop over 5 times
    cont=1;

    MediamMask=[-col-1, -col, -col+1, -1, 1, col-1, col, col+1];
%   MediamFilter Mask
% Note: mediam filter is valid only for ``allpts''
    IdxWildPoint=[];
    ind=find(abs(err)<meanerr);     
    while meanerr>thr & cont<=MaxLoop
        cont=cont+1;
%         for cnt1=1:row
%             Mask=MediamMask+cnt1;
%             IndValidMediamMask=find(Mask>=1 & Mask<=row);
%             
%             ValidMediamMask=Mask(IndValidMediamMask);
%             %the Index of the mediam filter mask
%             
%             SurroundSmallErrNum=sum(allErr(ValidMediamMask)<meanerr);
%             %SurroundSmallErrNum: how many cells surrounding 
%             % a specific point have error less than the mean value
%             if SurroundSmallErrNum<3
% %current point is a wile point, if less than 3 neighbour points
% % fall on this plane.
%                 IdxWildPoint=[IdxWildPoint, cnt1];
%             end
%         end
        
        [U, S, V]=svd([fitpts, ones(row,1)],0); 
        para=V(:,4)';    
        err=[fitpts,ones(row,1)]*para';                
        meanerr=mean(err);  
        
        ind=find(abs(err)<meanerr); 
%        ind=takeOffWildIdx(ind, IdxWildPoint);
        fitpts=fitpts(ind,:); 
        %if `thr' is set, pick out wild values and fit again
      
    end
        
%    TotalErr=sum( abs( [fitpts, ones(row,1)] * para'), 1);
%    meanerr=TotalErr/row;
    fitptidx=ind;
    return;    
end

function   ans=takeOffWildIdx(ind, IdxWildPoint)
    LenInd=length(ind);
%    LenWild=length(IdxWildPoint);
    ans=[];
    for cnt=1:LenInd
        if length(find(IdxWildPoint==ind(cnt)))==0
%This ``ind(cnt)'' does not contain any member in IdxWildPoint 
            ans=[ans, ind(cnt)];
        end
    end
end