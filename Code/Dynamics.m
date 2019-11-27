%% Drone Dynamics
clear
clc
% Variables
syms s1 s2 s3 v1 v2 v3 phi theta psi p q r T n1 n2 n3
pos = [s1;s2;s3];
vel = [v1;v2;v3];
orien = [phi;theta;psi];
rates = [p;q;r];
u = [T;n1;n2;n3];
mass = 32e-03;
J = (10^-6)*[16 16 29];
g = 9.81;

%% State Space
A = [0 0 0 1 0 0 0 0 0 0 0 0
     0 0 0 0 1 0 0 0 0 0 0 0
     0 0 0 0 0 1 0 0 0 0 0 0
     0 0 0 0 0 0 0 g 0 0 0 0
     0 0 0 0 0 0 -g 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 1 0 0
     0 0 0 0 0 0 0 0 0 0 1 0
     0 0 0 0 0 0 0 0 0 0 0 1
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0];
 
 B = [0 0 0 0
      0 0 0 0
      0 0 0 0
      0 0 0 0
      0 0 0 0
      1/mass 0 0 0
      0 0 0 0
      0 0 0 0
      0 0 0 0
      0 1/J(1) 0 0
      0 0 1/J(2) 0
      0 0 0 1/J(3)];
  
C = eye(12);

D = 0;

E = [0;0;0;0;0;0;0;0;0;0;0;0];
  
%% System
system = ss(A,B,C,D);
% Discretization 
dsys = c2d(system,0.1,'foh');
A = dsys.A;
B = dsys.B;
C = dsys.C;
D = dsys.D;
  
