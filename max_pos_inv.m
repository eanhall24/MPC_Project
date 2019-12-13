function [Oinf,converged] = max_pos_inv(Acl,S)
% INPUT ARGUMENTS
%   Acl : LTI, Discrete-time Closed-loop System x(k+1) = Acl*x(k)
%   S   : Target Set
% OUTPUT ARGUMENTS
%  Oinf : Maximal Positive Invariant Set
%  converged : 1 if converged within max iterations, 0 else

maxIterations = 500;  % user setting
Omega0 = S; % initialization
for i = 1 : maxIterations
    % compute backward reachable set
    P = Pre_Aut(Acl,Omega0);
    % intersect with the state constraints
    P = P.intersect(Omega0).minHRep();
    if P == Omega0
        Oinf = Omega0;
        break
    else
        Omega0 = P;
    end
end
if i == maxIterations
    converged = 0;
else
    converged = 1;
end
end