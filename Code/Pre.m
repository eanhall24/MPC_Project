function PreS = Pre(A,B,S,U)
% INPUT ARGUMENTS
%   A : LTI, Discrete-time System x(k+1) = A*x(k) + B*u(k)
%   B : LTI, Discrete-time System x(k+1) = A*x(k) + B*u(k)
%   S : Target Set
%   U : Input Constraint Set
% OUTPUT ARGUMENTS
%  PreS : Pre set for a driven system

nx = size(A,2);
nu = size(B,2);

H  = S.H(:,1:nx);
h  = S.H(:,nx+1);
Hu = U.H(:,1:nu);
hu = U.H(:,nu+1);
He = S.He(:,1:nx);
he = S.He(:,nx+1);

XU_H = [H*A H*B; zeros(size(Hu,1),nx) Hu];
XU_h = [h;hu];

XU = Polyhedron('H',[XU_H, XU_h],'He',[He*A, He*B, he]);

PreS = XU.projection(1:nx);
end