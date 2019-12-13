function [xOpt,uOpt,feas] = solver(A, B, P, Q, R, N, x0, xL, xU, uL, uU, bf, Af)
yalmip('clear')
nx = size(A,2);
nu = size(B,2);
x = sdpvar(nx,N+1);
u = sdpvar(nu,N);

m = 32e-03;
g = 9.81;

feas = false;

constr = [x(:,1) == x0];

if isempty(Af)
    constr = [constr, x(:,N+1)==bf];
else
    constr = [constr, Af*x(:,(N-5):(N+1))<=bf];
end

cost = x(:,N+1)'*P*x(:,N+1);

for k = 1:N
    constr = [constr, xL <= x(:,k),x(:,k)<= xU];
    
    constr = [constr, uL <= u(:,k),u(:,k) <= uU];
    
    constr = [constr, x(:,k+1) == A*x(:,k) + B*u(:,k)];
    
    cost = cost + (x(:,k) - x(:,N+1))'*Q*(x(:,k) - x(:,N+1)) + (u(:,k)-.25*m*g*ones(4,1))'*R*(u(:,k)-.25*m*g*ones(4,1));
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
