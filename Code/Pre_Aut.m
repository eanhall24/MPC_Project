function PreS = Pre_Aut(A,S)
% INPUT ARGUMENTS
%   A : LTI, Discrete-time Autonomous System x(k+1) = A*x(k)
%   S : Target Set
%   Note: also works with polytope which are not full dimensional
% OUTPUT ARGUMENTS
%   PreS : Pre set for an autonomous system

nx = size(A,2);

H  = S.H(:,1:nx);
h  = S.H(:,nx+1);
He = S.He(:,1:nx);
he = S.He(:,nx+1);

PreS = Polyhedron('H',[H*A, h],'He',[He*A, he]);
end