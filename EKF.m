% Localization of mobile Robot using Extended Kalman Filter in 2D space
% Matlab code
% Developed by: Bhanu Chander V, IIT Madras, India

clear; close all; clc

data = load('Robot_Path_Coordinates.txt');

% Required/desired x,y coordinates of the path 
% as planned by path planning algorithm

X = data(:,1); 
Y = data(:,2);
m = size(X,1);

warning("off",'all'); %To avoid warnig message of"Division by 0"

% Assuming all the noises are zero mean and are 
% uncorrelated with each other

% Measurements are assumed as the desired pose with some noise
% i.e required x,y,phi with some noise 0
% I skipped using a robot for taking measurements

% Initializing some Jacobians and other matrices

% Initial covarience matrix of pose (position & orientation)
p_correct = [1500 0 0; 0 1500 0; 0 0 17]; 

U = [1, 0; 0, 0.17]; % covarience matrix of control input
% 10 degree variation is considered in variance
Q = [1 0 0; 0 1 0; 0 0 0.17]; % covarience for prediction step
H = [1 0 0; 0 1 0; 0 0 1];
R = [0.2 0 0; 0 0.4 0; 0 0 0.03]; % covariance of the range sensor

% Varying R is very imp, BE CAREFUL !
% K = zeros(3);

% x = zeros(3,1);

% d = sqrt((X2-X1)^2+(Y2-Y1)^2));
% phi = atan((Y2-Y1)/(X2-X1));

[d, theta] = measurement(X(2),Y(2),X(1),Y(1)); %initial control

% intital start position of Robot
x(1,1) = X(1);
x(2,1) = Y(1);
x(3,1) = theta;

%disp(x);

hor = [100];
ver = [100];

% control inputs are euclidean distance d, and the heading angle (phi)

u(1) = d + normrnd(0,U(1,1)); 
u(2) = normrnd(0,U(2,2));


%disp(u(1)); 
%disp(u(2));

for step = 1:m-1;
	% u(1) = d + normrnd(0,U(1,1)); 
	% u(2) = phis+normrnd(0,U(2,2)); %this should be at the end ?
	
	% Prediction step
	x(1,1) += u(1)*cos(x(3,1)+u(2)) + normrnd(0,Q(1,1));
	x(2,1) += u(1)*sin(x(3,1)+u(2)) + normrnd(0,Q(2,2));
	x(3,1) += u(2) + normrnd(0,Q(3,3)); 
	%disp('estimate x value');
	%disp(x);
	
	A1 = [1, 0, -u(1)*sin(x(3,1)+u(2)/2); 
		 0, 1, u(1)*cos(x(3,1)+u(2)/2);
		 0, 0, 1];
	A2 = [cos(x(3,1)+u(2)/2), u(1)*sin(x(3,1)+u(2)/2); 
		 sin(x(3,1)+u(2)/2), -u(1)*cos(x(3,1)+u(2)/2);
		 0, 1];
	%disp('p_correct'),
	%disp(p_correct), 
	
	p_predict = (A1*p_correct*A1') + (A2*U*A2') + Q; 
	
	%disp('p_predict'),
	%disp(p_predict);
	%disp('\n');
	
	% In calculating actual measurement taken by range sensor
	% (either laser or sonar), we assume that the output is 	% accurate and the measurements are very close the actual 	% value, which is shown by adding small noise for simplicity 
	
	% In actual practice, this is the input from range sensors

	[d1, ph1]=measurement(X(step+1),Y(step+1),X(step),Y(step));
	Z = [X(step) + d1*cos(ph1) + normrnd(0,R(1,1));
		Y(step) + d1*sin(ph1) + normrnd(0,R(2,2));
		ph1 + normrnd(0,R(3,3))];
	%disp(ph1);
	%disp(Z),
	%disp('\n');

	% Correction steps
	
	K = p_predict*H'*inv(H*p_predict*H' + R); % Kalman Gain
	x = x +  K*(Z - H*x);
	
	%disp('corrected x value');
	%disp(x);
	p_correct = (eye(3) - K*H)*p_predict;
	
	%disp(d); %disp(theta);
	if(step < m-1);	
		
		[d, theta]= measurement(X(step+2),Y(step+2),x(1,1),x(2,1));
		%disp(d); 
		%disp(theta);
		u(1) = d + normrnd(0,U(1,1)); 
		u(2) = theta - x(3,1) + normrnd(0,U(2,2));
	end;
	hor = [hor x(1,1)];
	ver = [ver x(2,1)];
end;

% To display in scatter plot format
 
%if(0)
figure; 
hold on;
title('Path of the Robot in 2D space (top view)');
xlabel('Longitudinal movement');
ylabel('Lateral movement');
plot(X, Y,'+');
plot(hor, ver,'o',"color",'r');
hold off;
%end

% pause;

% To display in Constinuous line (path) format

if(0)
figure; 
hold on;
title('Path of the Robot in 2D space (top view)');
xlabel('Longitudinal movement');
ylabel('Lateral movement');
plot(X, Y);
plot(hor, ver,"color",'r');
hold off;
end
