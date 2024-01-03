global M; M=1000;
global m1; m1=100;
global m2; m2=100;
global l1; l1=20; 
global l2; l2=10; 
global g; g=9.81;
 
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
 %Initial state
 X_init=[
         0 ;
         0 ;
         0.5 ;
         0 ;
         0.6 ;
         0
     ];
 %Q and R values
 Q=[10 0 0 0 0 0;
 0 10 0 0 0 0;
 0 0 100 0 0 0;
 0 0 0 1 0 0;
 0 0 0 0 100 0;
 0 0 0 0 0 1];
 R=0.001;
 %C is the identity matrix which directly gives the output and D is 0
 C=eye(6);D=0;
 init_sys = ss(A , B , C , D) ;
 figure
 initial(init_sys , X_init)
 % After here, we make the LQR controller
 [K , P , Poles] = lqr(A , B , Q , R) ;

 lqr_controller = ss((A - (B*K)) , B , C , D) ;
 figure
 initial(lqr_controller , X_init)

 % Non linear controls
range = 0:0.1:500;
[t_nl, dx_nl] = ode45(@(t,x)orig_sys(t,x,-K*x), range, X_init);
figure
plot(t_nl, dx_nl(:,1))
xlabel('time(s)')
ylabel('x(m)')
title('Nonlinear control')



function dX = orig_sys(t, X, f)
global M;
global m1;
global m2;
global l1;
global l2;
global g;
% Get state values
x = X(1); dx = X(2); theta1 = X(3); dtheta1 = X(4); theta2 = X(5); dtheta2 = X(6);


% Add observer correction for each term
% Calculate terms using nonlinear equations of motion
dX = zeros(6,1);
dX(1) = dx;
dX(2) = (f-((m1*sin(theta1)*cos(theta1))+ ...
        (m2*sin(theta2)*cos(theta2)))*g-(l1*m1*(dX(3)^2)*sin(theta1))- ...
        (l2*m2*(dX(5)^2)*sin(theta2)))/(m1+m2+M-(m1*(cos(theta1)^2))- ...
        (m2*(cos(theta2)^2)));
dX(3) = dtheta1;
dX(4) = ((cos(theta1)*dX(2)-g*sin(theta1))/l1);
dX(5) = dtheta2;
dX(6) = (cos(theta2)*dX(2)-g*sin(theta2))/l2;
end