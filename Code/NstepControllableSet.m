function K = NstepControllableSet(A,B,Xf,X,U,N)
% INPUT ARGUMENTS
%   A : LTI, Discrete-time System x(k+1) = A*x(k) + B*u(k)
%   B : LTI, Discrete-time System x(k+1) = A*x(k) + B*u(k)
%   Xf: Terminal Set
%   X : State Constraint Set
%   U : Input Constraint Set
%   N : Number of steps
% OUTPUT ARGUMENTS
%   K : K(N) is the initial feasible set \mathcal{X}_0

KK = Xf;  % K_0(S)
for j = 1:N
    PreK = Pre(A,B,KK,U);
    KK = PreK.intersect(X);
    K(j) = KK;
end
end