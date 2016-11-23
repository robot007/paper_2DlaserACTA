function [angest,best] = isotropicLMS(x,y)
%function [angest,best] = isotropicLMS(x,y)
% To fit a line in the format of: y=m*x+b
% Angest: Angle estimation, m=tan(Angest)
% Best: b=Best
%----------------------------------------------------------------------------------
%
% M-FILE:				   isotropicLMS.m
% DESCRIPTION:		Finds the line through the data points that minimizes the error NORMAL to the line,
%								not necessarily in the y-direction (which is what the regular least squares curve fit does).
%								
% PROGRAMMER:	 Morgan Davidson
% WRITTEN:			   9 MAR 2001
% LAST MOD.:		  9 MAR 2001
%
% MODIFIED:      Zhen Song
% DATE :         26 Nov 2002
% DIFFERENCE:   1. Deleted figure display
%               2. Return best fit line parameters
%----------------------------------------------------------------------------------

% Begin isotropic LMS fit algorithm:
xbar = mean(x);
ybar = mean(y);

xprime = x - xbar;		% Shifting coordinates to the centroid of the data.
yprime = y - ybar;

ScatterMatrix = zeros(2,2);	% Finding the scatter matrix.
for i=1:length(x)
	ScatterMatrix = ScatterMatrix + [ xprime(i) yprime(i) ]' * [ xprime(i) yprime(i) ];
end
CovarianceMatrix = ScatterMatrix / ( length(x) - 1 );

% The eigenvectors of the scatter matrix point in the direction of the line and normal to it,
% while the eigenvalues give the variance in those directions.
[V,D] = eig( CovarianceMatrix );
std1 = sqrt(D(1,1));
std2 = sqrt(D(2,2));
eigvec1 = std1* V(:,1) / norm(V(:,1));
eigvec2 = std2 * V(:,2) / norm(V(:,2));

% The long axis isn't white-noise distributed, so tweak the variance to taste.
% and find the estimated m & b line parameters:
if std1>std2	% 1 is the long one.
	eigvec1 = eigvec1;
	eigvec2 = eigvec2;
	mest = eigvec1(2)/eigvec1(1);
	angle = atan2(eigvec1(2),eigvec1(1));
	normalSTD = std2;
else	% 2 is the long one.
   eigvec1 = eigvec1;
   eigvec2 = eigvec2;
	mest = eigvec2(2)/eigvec2(1);
	angle = atan2(eigvec2(2),eigvec2(1));
	normalSTD = std1;
end
	
data = [x' y'];

RotMat = [ cos(angle) sin(angle) ; -sin(angle) cos(angle) ];

j = 1;
for i=1:length(data)
	data(i,:) = (RotMat*(data(i,:)' - [xbar ybar]'))';
	
	if abs(data(i,2)) < 2*normalSTD
		newData(j,:) = data(i,:);
		j = j+1;
	end
	
end

for i=1:length(newData)
	newData(i,:) = (RotMat'*newData(i,:)')' + [xbar ybar];
end


% Begin isotropic LMS fit algorithm:
newxbar = mean(newData(:,1));
newybar = mean(newData(:,2));

newxprime = newData(:,1) - newxbar;		% Shifting coordinates to the centroid of the data.
newyprime = newData(:,2) - newybar;

newScatterMatrix = zeros(2,2);	% Finding the new scatter matrix.
for i=1:length(newData)
	newScatterMatrix = newScatterMatrix + [ newxprime(i) newyprime(i) ]' * [ newxprime(i) newyprime(i) ];
end
newCovarianceMatrix = newScatterMatrix / ( length(newData) - 1 );

% The eigenvectors of the scatter matrix point in the direction of the line and normal to it,
% while the eigenvalues give the variance in those directions.
[V,D] = eig( newCovarianceMatrix );
newstd1 = sqrt(D(1,1));
newstd2 = sqrt(D(2,2));
neweigvec1 = std1* V(:,1) / norm(V(:,1));
neweigvec2 = std2 * V(:,2) / norm(V(:,2));
	
% The long axis isn't white-noise distributed, so tweak the variance to taste.
% and find the estimated m & b line parameters:
if newstd1>newstd2	% 1 is the long one.
	mest = neweigvec1(2)/neweigvec1(1);
	angle = atan2(neweigvec1(2),neweigvec1(1));
	best = newybar - newxbar*neweigvec1(2) / neweigvec1(1) ;
else	% 2 is the long one.
	mest = neweigvec2(2)/neweigvec2(1);
	angle = atan2(neweigvec2(2),neweigvec2(1));
	best = newybar - newxbar*neweigvec2(2) / neweigvec2(1);
end
angest=angle;
return;