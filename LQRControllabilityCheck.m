M = 1000 ;
m1 = 100;
m2 = 100;
l1 = 20 ;
l2 = 10 ;
g  = 9.8 ;
A=[0 1 0 0 0 0;
 0 0 -(m1*g)/M 0 -(m2*g)/M 0;
 0 0 0 1 0 0;
 0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
 0 0 0 0 0 1;
 0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];
 
 disp('A matrix')
 disp(A)

 B=[ 0 ;
     1/M ;
     0 ;
     1/(M*(l1)) ;
     0 ;
     1/(M*l2)] ;
 disp('B matrix')
 disp(B)
 Controlab_m=[B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B] ;
 disp('Controllability matrix')
 disp(Controlab_m)
 disp('If the rank of the controllability matrix turns out to be 6 then after the values plugged in then the system is controllable')
 ra = rank(Controlab_m);
 if ra < 6 
     disp('Matrix is uncontrollable')
 end
 if ra == 6 
     disp('Matrix is controllable')
 end