%Variablesymbolsareasf
 syms M m1 m2 l1 l2 g;
 %Statespacerepresentationofthelinearisedmodel
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
 
 disp('controlability matrix')
 disp(Controlab_m)

 disp('Determinant of controlability matrix')
 disp(simplify(det(Controlab_m)));
 disp('In the above matrix if l1 = l2 then the determinant is zero means the condition here is that l1 not equal to l2')


 disp('Rank of controlability matrix')
 disp(rank(Controlab_m))

 disp('Setting l1 = l2 in the controllability matrix')
 Uncontrollable_matrix=subs(Controlab_m,l1,l2);
 disp('Displaying the rank of uncontrollable matrix')
 disp(rank(Uncontrollable_matrix))