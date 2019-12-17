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
f_wind = @(z) 0.08*z;
  
%% System
system = ss(A,B,[],[]);
% Discretization 
dsys = c2d(system,0.1,'zoh');
A = dsys.A;
B = dsys.B;
C = dsys.C;
D = dsys.D;

uRef = mass*g/4*ones(4,1); % For other kind of final states run optimization from slides

%% Model mismatch:

mass_tilda = 0.688*1.5;
l_tilda = 0.12*1.5;
k_tilda = 0.014*1.5;
J = (10^-4)*[36 36 74];
g = 9.81;

%% State Space
A_tilda = [0 1 0 0 0 0 0 0 0 0 0 0
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

 B_tilda = [0 0 0 0 0
      0 0 0 0 0
      0 0 0 0 0 
      -l_tilda/J(2) -l_tilda/J(2) l_tilda/J(2) l_tilda/J(2) 0
      0 0 0 0 0
      0 0 0 0 0
      0 0 0 0 0
      l_tilda/J(1) -l_tilda/J(1) -l_tilda/J(1) l_tilda/J(1) 0
      0 0 0 0 0
      1/mass_tilda 1/mass_tilda 1/mass_tilda 1/mass_tilda -1
      0 0 0 0 0
      k_tilda/J(3) -k_tilda/J(3) k_tilda/J(3) -k_tilda/J(3) 0];
  
%% System
system_tilda = ss(A_tilda,B_tilda,[],[]);
% Discretization 
dsys_tilda = c2d(system_tilda,0.1,'zoh');
A_tilda = dsys_tilda.A;
B_tilda = dsys_tilda.B;
