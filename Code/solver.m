function [xOpt,uOpt,feas] = solver(A, B, P, Q, R, N, x0, xL, xU, uL, uU, bf, Af, Xd, uRef)
yalmip('clear')
nx = size(A,2);
nu = size(B,2);
x = sdpvar(nx,N+1);
u = sdpvar(nu,N);

feas = false;

constr = [x(:,1) == x0];

if isempty(Af)
%     constr = [constr, x(:,N-1)==bf, x(:,N)==bf, x(:,N+1)==bf];
    constr = [constr, x(:,N+1)==bf];
else
%     constr = [constr, Af*x(:,(N-5):(N+1))<=bf];
    constr = [constr Af*x(:,N+1)<=bf];
end

cost = (x(:,N+1)-Xd)'*P*(x(:,N+1)-Xd);

for k = 1:N
    constr = [constr, xL <= x(:,k),x(:,k)<= xU];
    
    constr = [constr, uL <= u(:,k),u(:,k) <= uU];
    
    constr = [constr, x(:,k+1) == A*x(:,k) + B*u(:,k)];

%     cost = cost + (x(:,k) - x(:,N+1))'*Q*(x(:,k) - x(:,N+1)) + (u(1:4,k))'*R*(u(1:4,k)); % Add last input (u-uref)
    cost = cost + (x(:,k) - Xd)'*Q*(x(:,k) - Xd) + (u(1:4,k)-uRef)'*R*(u(1:4,k)-uRef);
end

options = sdpsettings('verbose',0,'solver','quadprog');
sol = optimize(constr,cost,options);

% assign(x,0);
% check(constr)

if sol.problem == 0
    feas = true;
else
    xOpt = [];
    uOpt = [];
    return;
end

xOpt = double(x);
uOpt = double(u);
