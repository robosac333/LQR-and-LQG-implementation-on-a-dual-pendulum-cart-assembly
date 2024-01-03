# LQR-and-LQG-implementation-on-a-dual-pendulum-cart-assembly
ENPM 667 Controls project to test controllability and observability and apply linear and non-linear control to the assembly. 
This is the final project of the course ENPM667 Controls of Robotics System of M Eng. Robotics Program at University of Maryland, College Park The problem statement is given at the link below:
https://drive.google.com/file/d/1vP15PRw1K_9l1SpAUY_fKMjI-yDJYMBU/view?usp=sharing
Problem statement:

![image](https://github.com/robosac333/LQR-and-LQG-implementation-on-a-dual-pendulum-cart-assembly/assets/143353582/3a5c3a15-1b81-4d19-8221-f1d68cdad766)

Process followed to solve this assigment:

1. Obtained the equations of motion for the system and the corresponding nonlinear state-space representation.
2. Obtained the linearized system around the equilibrium point specified by x = 0 and θ1 = θ2 = 0. Writeen the state-space representation of the linearized system.
3. Obtained conditions on M, m1, m2, l1, l2 for which the linearized system is controllable.
4. Checked thatthe system is controllable and obtained an LQR controller.
5. Simulated the resulting response to initial conditions when the controller is applied to the linearized system and also to the original nonlinear system.
6. Determined for which output vectors the linearized system is observable.
7. Obtained best Luenberger observer for each one of the output vectors for which the system is observable and simulate its response to initial conditions and unit step input.
8. Designed an output feedback controller for the choice of output vector. Used the LQG method and apply the resulting output feedback controller to the original nonlinear system.
Technology used: MATLAB, Latex
