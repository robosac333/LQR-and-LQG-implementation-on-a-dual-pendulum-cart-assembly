% Analysis of Observability of various outputs

% System characteristics
M=1000; m1=100; m2=100; l1=20; l2=10; g=9.81;

% State Space Representation
% A and B Matrices
A=[0 1 0 0 0 0;
0 0 -(m1*g)/M 0 -(m2*g)/M 0;
0 0 0 1 0 0;
0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
0 0 0 0 0 1;
0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];

B=[0;
1/M;
0;
1/(M*(l1));
0;
1/(M*l2)];

% If the pair A^T C^T is controllable then A, C is observable
At = A.';
% C Matrix for each output type
C_1 = [1 0 0 0 0 0]; % x(t)
C_1t = C_1.';
C1Control = [C_1t At*C_1t At*At*C_1t At*At*At*C_1t At*At*At*At*C_1t At*At*At*At*At*C_1t];
disp("Controllability matrix rank for output set 1")
disp(rank(C1Control))

C_2 = [0 0 1 0 0 0;
       0 0 0 0 1 0]; % theta1, theta2
C_2t = C_2.';
C2Control = [C_2t At*C_2t At*At*C_2t At*At*At*C_2t At*At*At*At*C_2t At*At*At*At*C_2t];
disp("Controllability matrix rank for output set 2")
disp(rank(C2Control))

C_3 = [1 0 0 0 0 0;
       0 0 0 0 1 0]; % x, theta2
C_3t = C_3.';
C3Control = [C_3t At*C_3t At*At*C_3t At*At*At*C_3t At*At*At*At*C_3t At*At*At*At*C_3t];
disp("Controllability matrix rank for output set 3")
disp(rank(C3Control))

C_4 = [1 0 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 1 0]; % x, theta1, theta2
C_4t = C_4.';
C4Control = [C_4t At*C_4t At*At*C_4t At*At*At*C_4t At*At*At*At*C_4t At*At*At*At*C_4t];
disp("Controllability matrix rank for output set 4")
disp(rank(C4Control))