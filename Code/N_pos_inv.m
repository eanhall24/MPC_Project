function [Oinf,converged]=N_pos_inv(Acl,X)

N = 50;
Omega0 = X; % initialization
for i = 1:N
    % compute backward reachable set
    P = Pre_Aut(Acl,Omega0);
    % intersect with the state constraints
    P = P.intersect(Omega0).minHRep();
    if P==Omega0
        Oinf=Omega0;
        break
    else
        Omega0 = P;
    end
end
Oinf = Omega0;
% if i==maxIterations,
%     converged=0;
% else
%     converged=1;
% end
end



