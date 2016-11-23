% SPHT Sparse Hough Transform
% SPHT(DAT,PERCENT,RES,OPT)
% DAT      : input data, in the format of (x,y)
% PERCENT  : threashold for the accumulator space
% RES      : resolution [angular resolution(degree), range resolution(cm)]
% OPT 'cmp': draw comparison diagram 
%     'cal': calculate 
% 
%
% ver 1.0
% Zhen Song zhensong@cc.usu.edu
function [Ang,Rho,MemSz,Time]=spht(dat,percent,res,opt)
    k=1:100;
    n=50;
    m=50;
if strcmp(opt,'cmp');  %compare memory and computation
    Ang=0; Rho=0; MemSz=0;Time=0;
    
    disp('You have selected cmp option, which will show you the comparsion of SHT, SPHT and logHT (LHT)');
    disp('Your input data is invalid now. Let us consider the following case:');
    disp('Distance 50cm to 10m, resolution 1cm/0.5 degree. Scan 0 to 360 degree');
    dist=[50:200];
    AngRes=0.5;
    len=length(dist);
    
    subplot(3,1,1);
    Points=[1:720];
    Degree=Points./2;
    SHTmem=Points.*(360/AngRes); %
    SPHTmem=Points.*(Points-1)/2*3; %
    LHTmem=338*Points;
    h1=plot(Degree,SHTmem);
    hold on;
    h2=plot(Degree,SPHTmem,':');
    h3=plot(Degree,LHTmem,'-.');
    xlabel('Scanning Angle(degree)');
    ylabel('Memory Required');
    legend([h1,h2,h3],'SHT','SPHT','LHT');
    title('Memory Requirement Comparision');
    
    len=length(Points);
    SHTcomp=Points*(360/AngRes);
    SPHTcomp=Points.*(Points-1)/2;
    LHTcomp=Points;
    axis([0, max(dist),0,1.5e5]); 
    subplot(3,1,2);
    hold on;
    h1=plot(Degree,SHTcomp);
    h2=plot(Degree,SPHTcomp,':');
    h3=plot(Degree,LHTcomp,'-.');
    xlabel('Scanning Angle(degree)');
    ylabel('Computation Complexity');
    legend([h1,h2,h3],'SHT', 'SPHT','LHT');
    title('Complexity Comparison ');
    
    subplot(3,1,3);
    hold on;
    dist=[100:0.1:1000];
    r1=1000;
    r0=50;
    len=length(dist);
    HTres=0.1*ones(1,len);
    N=ceil(log((max(dist)-r0)/50)./(AngRes*pi/180));%how many point to provide the resolution
    n=ceil( log((dist-r0)/r0)./  (log((r1-r0)/r0)/(N-1))    ) ; %log(max((dist)-min(dist))/N) is one segment in log space
    LHTres=exp((n+1)*AngRes*pi/180)-exp(n*AngRes*pi/180);
    LHTres=LHTres*100; %m -> cm
    h2=plot(dist,HTres);
    h1=plot(dist,LHTres,'-.');
    legend([h1,h2],'LHT','SHT SPHT');
    xlabel('Distance(cm)');
    ylabel('Resolution(cm)');
    axis([0 1000, -1,20]);
    title('Resolution Comparision');
    
    
elseif strcmp(opt,'cal');%calculate
    tic;%start timer
    %   dat is the input data
    [Len,Col]=size(dat);
    if Col~=2
        disp('dat must be n by 2 matrix');
    end
    ResTheta=0.0175*res(1); %resolution of theta: 1 degree or 0.0175 rad
    ResRho=0.01*res(2); %resolution of rho: 0.01 m
    
    MyLines=zeros(1,5); %MyLines = [Overlap, Theta, Rho,  CellTheta, CellRho]
    for cnt1=1:Len-1
        for cnt2=cnt1+1:Len
            if abs(dat(cnt1,1)-dat(cnt2,1))<1e-6
                alpha=pi/2; % do not distinguish +-pi/2, since there is no direction of the line
            else 
                alpha=atan( (dat(cnt1,2)-dat(cnt2,2))/(dat(cnt1,1)-dat(cnt2,1)) );
            end
            rho0=dat(cnt1,1)*cos(alpha-pi/2) + dat(cnt1,2)*sin(alpha-pi/2);
            Theta=alpha-sign(rho0)*pi/2;
            CellTheta=floor(Theta/ResTheta)*ResTheta;
            Rho=abs(rho0); % dat(cnt2,1)*cos(Theta)+dat(cnt2,2)*sin(Theta);
            CellRho=floor(Rho/ResRho)*ResRho;
            
            Rep=find(MyLines(:,4)==CellTheta & MyLines(:,5)==CellRho);
            if length(Rep)~=0
                MyLines(Rep,1)=MyLines(Rep,1)+1;
            else
                MyLines=[MyLines; 1, Theta, Rho, CellTheta, CellRho];
            end
        end
    end
    [LineNum,tmp]=size(MyLines);
    LineNum=LineNum-1; %there is garbage at the beginning
    MyLines=MyLines(2:LineNum+1,:);
    %find match

    MaxVote=max(MyLines(:,1));
    VotedInd=[];
    VotedInd=find(MyLines(:,1)>percent*MaxVote);

    Total=length(VotedInd);
    
    figure;
    title('SPHT: Fitted Lines');
    hold on;
    grid on;
    axis ([-1 1 -1 1])
    x=-1:0.01:1;
    for cnt1=1:Total
        Ang=MyLines(VotedInd(cnt1),4);
        Rh= MyLines(VotedInd(cnt1),5);
        y=(-cos(Ang)/sin(Ang))*x+Rh/sin(Ang);
        plot(x,y);
    end
    MemSz=size(MyLines)
    Time=toc
% if you want to see 3D accumulator space, you and uncomment the following lines

%     figure;
%     plot3(MyLines(:,4),MyLines(:,5),MyLines(:,1),'o');
%     grid on;
    
    figure;
    hold on;
    title('SPHT: Data and the Fitted Lines');
    h1=plot(dat(:,1),dat(:,2),'o');
    max_x=max(dat(:,1));  min_x=min(dat(:,1)); max_y=max(dat(:,2)); min_y=min(dat(:,2));
    height=max_y-min_y; width=max_x-min_x;
    grid on;
    axis equal;
    axis([min_x-0.2*width, max_x+0.2*width, min_y-0.2*height, max_y+0.2*height]); %adjust the display area.
    for cnt1=1:Total
        Ang=MyLines(VotedInd(cnt1),4);
        Rh= MyLines(VotedInd(cnt1),5);
        y=(-cos(Ang)/sin(Ang))*x+Rh/sin(Ang);
        h2=plot(x,y,'r-.');
    end
    legend([h1, h2],'Raw Data','Fitted Line');
   
end

