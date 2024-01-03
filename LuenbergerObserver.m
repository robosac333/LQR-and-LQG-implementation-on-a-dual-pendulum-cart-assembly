% Luenberg Observer

% System characteristics
global M; M=1000;
global m1; m1=100;
global m2; m2=100;
global l1; l1=20; 
global l2; l2=10; 
global g; g=9.81;

% set some initial condition, just need to start with state estimate err=0
% Angles have some initial value in system image
x_zero = [0; 0; pi/4; 0; pi/3; 0; 0; 0; 0; 0; 0; 0];

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

% Get the LQR Controller from Part D.
% Need K for observer
Q=[10 0 0 0 0 0;
0 10 0 0 0 0;
0 0 100 0 0 0;
0 0 0 1 0 0;
0 0 0 0 100 0;
0 0 0 0 0 1];
R=0.001;

[K , P , Poles] = lqr(A , B , Q , R) ;

% Observable C Matrices
C_1 = [1 0 0 0 0 0]; % x(t)
C_3 = [1 0 0 0 0 0;
       0 0 0 0 1 0]; % x, theta2
C_4 = [1 0 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 1 0]; % x, theta1, theta2

% Originally used this code to get poles but system has repeated poles and 
% poles in only imaginary space
% poles = eig(A)
% luenberg_poles = 5*poles
luenberg_poles = [-5.0;
                  -6.0;
                  -4.0;
                  -7.0
                  -8.0;
                  -3.0];

% Linear system observers

% System 1 Observer
% Poles are arbitrarily selected to be a large negative value
% L matrix derived from poles
L_c1 = place(A', C_1', luenberg_poles);

A_c1 = [A-B*K B*K; zeros(size(A)) A-L_c1.'*C_1];
B_c1 = [B;B];
C_c1 = [C_1 zeros(size(C_1))];
sys_c1 = ss(A_c1, B_c1, C_c1, 0);

disp("System 1 Observer");
disp(L_c1);
initialplot(sys_c1, x_zero)
figure()
stepplot(sys_c1)


% System 3 Observer
L_c3 = place(A', C_3', luenberg_poles);

A_c3 = [A-B*K B*K; zeros(size(A)) A-L_c3.'*C_3];
B_c3 = [B;B];
C_c3 = [C_3 zeros(size(C_3))];
sys_c3 = ss(A_c3, B_c3, C_c3, 0);

disp("System 3 Observer");
disp(L_c3);
figure()
initialplot(sys_c3, x_zero)
figure()
stepplot(sys_c3)

% System 4 Observer
L_c4 = place(A', C_4', luenberg_poles);

A_c4 = [A-B*K B*K; zeros(size(A)) A-L_c4.'*C_4];
B_c4 = [B;B];
C_c4 = [C_4 zeros(size(C_4))];
sys_c4 = ss(A_c4, B_c4, C_c4, 0);

disp("System 4 Observer");
disp(L_c4);
figure()
initialplot(sys_c4, x_zero)
figure()
stepplot(sys_c4)

% Closes all linear graphs, comment this out if you want to see them again
%close all

% Non linear system observers
% Same initial condition
x_zero_nl = [0; 0; pi/4; 0; pi/3; 0];
t_range = 0:0.01:500;

% System 1 Nonlinear
[t_1, dx_1] = ode45(@(t,x)orig_sys(t,x,L_c1,C_1,-K*x,1),t_range,x_zero_nl);

figure
plot(t_1,dx_1(:,1))
grid
xlabel('time(s)')
ylabel('x(m)')
title('Nonlinear System 1 Output')

% System 3 Nonlinear
[t_3, dx_3] = ode45(@(t,x)orig_sys(t,x,L_c3,C_3,-K*x,3),t_range,x_zero_nl);

figure
subplot(2,1,1)
plot(t_3,dx_3(:,1));
xlabel('time(s)');
ylabel('x(m)');
title('Nonlinear System 3 Output');
subplot(2,1,2)
plot(t_3,dx_3(:,5));
xlabel('time(s)');
ylabel('theta_2(rad)');
title('Nonlinear System 3 Output');

% System 4 Nonlinear
[t_4, dx_4] = ode45(@(t,x)orig_sys(t,x,L_c4,C_4,-K*x,4),t_range,x_zero_nl);

figure
subplot(3,1,1)
plot(t_4,dx_4(:,1));
xlabel('time(s)');
ylabel('x(m)');
title('Nonlinear System 4 Output');
subplot(3,1,2)
plot(t_4,dx_4(:,3));
xlabel('time(s)');
ylabel('theta_1(rad)');
title('Nonlinear System 4 Output');
subplot(3,1,3)
plot(t_4,dx_4(:,5));
xlabel('time(s)');
ylabel('theta_2(rad)');
title('Nonlinear System 4 Output');

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
