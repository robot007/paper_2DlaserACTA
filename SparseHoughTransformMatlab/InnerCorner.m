function [Corner, FitErr]=InnerCorner(NInPts,Area,Height, PFitThr)
%function [Corner, FitErr]=InnerCorner(NInPts,Area,Height,PFitThr)
% NInPts : a 3 by n matrix of data points
% Area : [Xmin; Xmax; Ymin; Ymax];
% Height: [MinHeight; MaxHeight]. the height range of the corner edges
% PFitThr: Plane fitting threshold. If it is empty, i.e., PFitThr=[],
%       then use the default fitting threshold 5e-3. (Note: this is 
%       algebric error, not geometry error)
%
%Assumptions:
% 1 Only a inner corner in the data set
% 2 The ground is flat, with noise less than the PFitThr
% 3 The corner is not necessarily a rectengular corner
% 
% Jan 14 05: Add fitting error return 
%
% Nov 02
% Zhen Song

Xmin=Area(1);
Xmax=Area(2);
Ymin=Area(3);
Ymax=Area(4);
CorRegin=[];
PtNum=size(NInPts,2);
for cnt=1:PtNum
    if NInPts(1,cnt)>Xmin & NInPts(1,cnt)< Xmax & NInPts(2,cnt)>Ymin & NInPts(2,cnt)<Ymax
        CorRegin=[CorRegin, NInPts(:,cnt)];
    end
end





%delete noise
idx=find(CorRegin(1,:)>3.4 & CorRegin(1,:)<3.8 & CorRegin(3,:)>-1.27)
CorRegin(3,idx)=-1.35;

%idx=find(CorRegin(1,:)>0.4 & CorRegin(3,:)>-1.27)
%CorRegin(3,idx)=-1.35;





%Plane Fit
if isempty(PFitThr)
    thr=1e-2; %10mm
else
    thr=PFitThr;
end
%[para, meanerr,fitptidx] = pfit(CorRegin',thr);
%PtNum=size(CorRegin,2);
%Err=abs([CorRegin',ones(PtNum,1)]*para');
%CorIdx=find(Err>thr); %all points that not on the plane
%InnerCor=CorRegin(:,CorIdx);
CorIdx=find(CorRegin(3,:)>-1.25);
InnerCor=CorRegin(:,CorIdx);

%LowH=min(Height);
%HighH=max(Height);
%CorHighIdx=find(InnerCor(3,:)<HighH & InnerCor(3,:)>LowH);
%LowH=mean(InnerCor(3,:));
%CorHighIdx=find(InnerCor(3,:)<Height(1)+LowH & InnerCor(3,:)>LowH+Height(2));

%InnerCor=InnerCor(:,CorHighIdx);%all points in certain height region
Dist2Cor=InnerCor(1,:).^2+InnerCor(2,:).^2+InnerCor(3,:).^2;
CorIdx=find(Dist2Cor==max(Dist2Cor));%the corner of the inner corner
CorY=InnerCor(2,CorIdx);
CorLeftIdx=find(InnerCor(2,:)<CorY);
CorLeft=InnerCor(:,CorLeftIdx);
CorRightIdx=find(InnerCor(2,:)>CorY);
CorRight=InnerCor(:,CorRightIdx);
[Rang,Rb]=isotropicLMS(CorRight(1,:),CorRight(2,:));
[Lang,Lb]=isotropicLMS(CorLeft(1,:),CorLeft(2,:));
Rm=tan(Rang);
Lm=tan(Lang);
RErr=abs(Rm*CorRight(1,:)+Rb-CorRight(2,:));
RSmallErr=find(RErr<mean(RErr));
CorRight=CorRight(:,RSmallErr);
LErr=abs(Lm*CorLeft(1,:)+Lb-CorLeft(2,:));
LSmallErr=find(LErr<mean(LErr));
CorLeft=CorLeft(:,LSmallErr);

[Rang,Rb]=isotropicLMS(CorRight(1,:),CorRight(2,:));
[Lang,Lb]=isotropicLMS(CorLeft(1,:),CorLeft(2,:));
Rm=tan(Rang);
Lm=tan(Lang);



if Rm==Lm
    disp('there is no intersection');
    Corner=[];
    return;
else
    para=[-Rm 1; -Lm 1];
    Corner=inv(para)*[Rb;Lb];
end


%% print errors for analysis
disp('The geometry fitting error:');
Rrho=Rb*cos(Rang); Rth=Rang+pi/2;
Rdis=abs( Corner(2)*sin(Rth)+Corner(1)*cos(Rth)-Rrho );
str=['The distance from corner to the right line: ', num2str(Rdis) ];
disp(str);

Lrho=Lb*cos(Lang); Lth=Lang+pi/2;
Ldis=abs( Corner(2)*sin(Lth)+Corner(1)*cos(Lth)-Lrho );
str=['The distance from corner to the left line: ', num2str(Ldis) ];
disp(str);


ESumR= mean( abs(CorRight(2,:) *sin(Rth)+CorRight(1)*cos(Rth)-Rrho ) );
ESumL= mean( abs(CorLeft(2,:) *sin(Rth)+CorLeft(1)*cos(Rth)-Rrho ) );
str=['Mean error for right edge: ', num2str(ESumR)];
disp(str);
str=['Mean error for left edge: ', num2str(ESumL)];
disp(str);
return;