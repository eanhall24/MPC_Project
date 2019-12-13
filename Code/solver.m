function [xOpt,uOpt,feas] = solver(A, B, P, Q, R, N, x0, xL, xU, uL, uU, bf, Af, xdes)
yalmip('clear')
nx = size(A,2);
nu = size(B,2);
x = sdpvar(nx,N+1);
u = sdpvar(nu,N);

% Do state transformation here
for i=1:N+1
    x(:,i) = x(:,i) - xdes ; 
end

feas = false;

constr = [x(:,1) == x0];

if isempty(Af)
    constr = [constr, x(:,N-1)==bf, x(:,N)==bf, x(:,N+1)==bf];
else
    for i=0
        constr = [constr, Af*x(:,N+1-i)<=bf];
    end
end

cost = x(:,N+1)'*P*x(:,N+1);

for k = 1:N
    constr = [constr, xL <= x(:,k),x(:,k)<= xU];
    
    constr = [constr, uL <= u(:,k),u(:,k) <= uU];
    
    constr = [constr, x(:,k+1) == A*x(:,k) + B*u(:,k)];

    cost = cost + x(:,k)'*Q*(x(:,k)) + (u(:,k))'*R*(u(:,k));
end

options = sdpsettings('verbose',1,'solver','quadprog');
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
