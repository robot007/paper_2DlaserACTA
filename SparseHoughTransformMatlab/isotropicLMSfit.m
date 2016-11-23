%function f = isotropicLMSfit (x,y)

%----------------------------------------------------------------------------------
%
% M-FILE:				   isotropicLMSfit.m
% DESCRIPTION:		Finds the line through the data points that minimizes the error NORMAL to the line,
%								not necessarily in the y-direction (which is what the regular least squares curve fit does).
%								
% PROGRAMMER:	 Morgan Davidson
% WRITTEN:			   9 MAR 2001
% LAST MOD.:		  9 MAR 2001
%
%----------------------------------------------------------------------------------

home, clear

x = 0:100;
y = eye(length(x),1);

m = 0		% For line equation y = mx + b.
b = 100

for i=1:length(x)-5
	y(i) = m*x(i) + b;		% Building my original line.
	
	x(i) = x(i) + 2*randn;	% Perturbing my original line.
	y(i) = y(i) + 2*randn;	
	
	if i > length(x) - 5	% Outlier points.
		x(i) = x(i) + 5*randn;
		y(i) = y(i) + 5*randn;
	end	
end	

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
eigvec1 = std1* V(:,1) / norm(V(:,1))
eigvec2 = std2 * V(:,2) / norm(V(:,2))

% The long axis isn't white-noise distributed, so tweak the variance to taste.
% and find the estimated m & b line parameters:
if std1>std2	% 1 is the long one.
	eigvec1 = eigvec1
	eigvec2 = eigvec2
	mest = eigvec1(2)/eigvec1(1);
	angle = atan2(eigvec1(2),eigvec1(1));
	normalSTD = std2
else	% 2 is the long one.
   eigvec1 = eigvec1
   eigvec2 = eigvec2
	mest = eigvec2(2)/eigvec2(1);
	angle = atan2(eigvec2(2),eigvec2(1));
	normalSTD = std1
end

% 
% figure(1),clf
% 	plot(x,y,'ko')
% 	axis equal
% 	hold on
% 	plot(xbar,ybar,'ro')
% 	quiver([xbar xbar],[ybar ybar],[eigvec1(1) eigvec2(1)],[eigvec1(2) eigvec2(2)],'r')
% 	
	
data = [x',y];
size(data);
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
	mest = neweigvec1(2)/neweigvec1(1)
	angle = atan2(neweigvec1(2),neweigvec1(1));
	best = newybar - newxbar*neweigvec1(2) / neweigvec1(1) 
	newstd1/newstd2
else	% 2 is the long one.
	mest = neweigvec2(2)/neweigvec2(1)
	angle = atan2(neweigvec2(2),neweigvec2(1));
	best = newybar - newxbar*neweigvec2(2) / neweigvec2(1)
	newstd2/newstd1
end

for i=1:length(x)
	xest(i) = i;
	yest(i) = mest*xest(i) + best;
end

%figure(2),clf
%	plot(newData(:,1),newData(:,2),'go')
%	axis equal
%	hold on
%	quiver([newxbar newxbar],[newybar newybar],[neweigvec1(1) neweigvec2(1)],[neweigvec1(2) neweigvec2(2)],'r')
% 	
% figure(1),
% 	plot(xest,yest,'b')
% 	plot(newxbar,newybar,'bo')
