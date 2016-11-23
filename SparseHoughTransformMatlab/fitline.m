function [t,r]=fitline(Pt1, Pt2)

if abs(Pt1(1)-Pt2(1))<1e-6
    alpha=pi/2; % do not distinguish +-pi/2, since there is no direction of the 
else 
%    alpha=atan( (dat(cnt1,2)-dat(cnt2,2))/(dat(cnt1,1)-dat(cnt2,1)) );
    alpha=atan( (Pt1(2)-Pt2(2))/(Pt1(1)-Pt2(1)) );
end
rho0=Pt1(1)*cos(alpha-pi/2) + Pt1(2)*sin(alpha-pi/2);
Theta=alpha-sign(rho0)*pi/2;
Rho=abs(rho0); % dat(cnt2,1)*cos(Theta)+dat(cnt2,2)*sin(Theta);
r=Rho;
t=Theta;
return;
%CellTheta=floor(Theta/ResTheta)*ResTheta;

%CellRho=floor(Rho/ResRho)*ResRho;
            