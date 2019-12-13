%% Drone Dynamics
clear
clc
% Variables
syms s1 v1 theta q s2 v2 phi p s3 v3 psi r T n1 n2 n3 cp1 cp2 cp3 cp4
pos = [s1;s2;s3];
vel = [v1;v2;v3];
orien = [phi;theta;psi];
rates = [p;q;r];
% u = [T;n1;n2;n3];
u = [cp1;cp2;cp3;cp4];
mass = 0.688;
l = 0.12;
k = 0.014;
J = (10^-4)*[36 36 74];
g = 9.81;

%% State Space
A = [0 1 0 0 0 0 0 0 0 0 0 0
     0 0 g 0 0 0 0 0 0 0 0 0
     0 0 0 1 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 1 0 0 0 0 0 0
     0 0 0 0 0 0 -g 0 0 0 0 0
     0 0 0 0 0 0 0 1 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 1 0 0
     0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 1
     0 0 0 0 0 0 0 0 0 0 0 0];
 
%  B = [0 0 0 0
%       0 0 0 0
%       0 0 0 0
%       0 0 0 0
%       0 0 0 0
%       1/mass 0 0 0
%       0 0 0 0
%       0 0 0 0
%       0 0 0 0
%       0 1/J(1) 0 0
%       0 0 1/J(2) 0
%       0 0 0 1/J(3)];

 B = [0 0 0 0 0
      0 0 0 0 0
      0 0 0 0 0 
      -l/J(2) -l/J(2) l/J(2) l/J(2) 0
      0 0 0 0 0
      0 0 0 0 0
      0 0 0 0 0
      l/J(1) -l/J(1) -l/J(1) l/J(1) 0
      0 0 0 0 0
      1/mass 1/mass 1/mass 1/mass -1
      0 0 0 0 0
      k/J(3) -k/J(3) k/J(3) -k/J(3) 0];
  
C = eye(12);

D = 0;

E = [0;0;0;0;0;-g;0;0;0;0;0;0];

G = zeros(12,1);
G(6) = 1;

% f_wind = @(z) 0.2*z^2;
f_wind = @(z) 0.15;
  
%% System
system = ss(A,B,[],[]);
% Discretization 
dsys = c2d(system,0.1,'zoh');
A = dsys.A;
B = dsys.B;
C = dsys.C;
D = dsys.D;

uRef = mass*g/4*ones(4,1); % For other kind of final states run optimization from slides

  
