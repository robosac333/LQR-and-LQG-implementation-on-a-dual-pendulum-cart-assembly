% Output Feedback Controller

% System characteristics
global M; M=1000;
global m1; m1=100;
global m2; m2=100;
global l1; l1=20; 
global l2; l2=10; 
global g; g=9.81;

% Initial pos
x_zero = [0; 0; pi/4; 0; pi/3; 0];

% A and B matrices
A=[0 1 0 0 0 0;
0 0 -(m1*g)/M 0 -(m2*g)/M 0;
0 0 0 1 0 0;
0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
0 0 0 0 0 1;
0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];

B=[ 0 ;
1/M ;
0 ;
1/(M*(l1)) ;
0 ;
1/(M*l2)] ;

C_1 = [1 0 0 0 0 0]; % x(t)

state_sp = ss(A, B, C_1, 0);

% Get the LQR Controller from Part D.
% Need K for observer
Q=[10 0 0 0 0 0;
0 10 0 0 0 0;
0 0 100 0 0 0;
0 0 0 1 0 0;
0 0 0 0 100 0;
0 0 0 0 0 1];
R=0.001;

% Getting K using LQR
[K , P , Poles] = lqr(A , B , Q , R) ;

% Getting L using Kalman
[kalmf, L, P] = kalman(state_sp,1,1,0);

% Apply K and L to nonlinear system
[t_1, dx_1] = ode45(@(t,x)orig_sys(t,x,L,C_1,-K*x,1),t_range,x_zero);

figure
plot(t_1,dx_1(:,1))
grid
xlabel('time(s)')
ylabel('x(m)')
title('Output Feedback Controller')

% Function for original nonlinear system
function dX = orig_sys(t, X, L, C, F, sys_num)
global M;
global m1;
global m2;
global l1;
global l2;
global g;
% Get state values
x = X(1); dx = X(2); theta1 = X(3); dtheta1 = X(4); theta2 = X(5); dtheta2 = X(6);
% For the different system states
switch sys_num
    case 1
        y = [x];
    case 3
        y = [x; theta2];
    case 4
        y = [x; theta1; theta2];
    otherwise
        y = [x];
end
% Observer correction term
obs_corr = L'*(y-C*X);

% Add observer correction for each term
% Calculate terms using nonlinear equations of motion
dX = zeros(6,1);
dX(1) = obs_corr(1) + dx;
dX(2) = obs_corr(2) + (F-((m1*sin(theta1)*cos(theta1))+ ...
        (m2*sin(theta2)*cos(theta2)))*g-(l1*m1*(dX(3)^2)*sin(theta1))- ...
        (l2*m2*(dX(5)^2)*sin(theta2)))/(m1+m2+M-(m1*(cos(theta1)^2))- ...
        (m2*(cos(theta2)^2)));
dX(3) = obs_corr(3) + dtheta1;
dX(4) = obs_corr(4) + ((cos(theta1)*dX(2)-g*sin(theta1))/l1);
dX(5) = obs_corr(5) + dtheta2;
dX(6) = obs_corr(6) + (cos(theta2)*dX(2)-g*sin(theta2))/l2;
end