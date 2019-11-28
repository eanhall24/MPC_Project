function [xOpt,uOpt,feas] = solver(A, B, P, Q, R, N, x0, xL, xU, uL, uU, bf, Af)
yalmip('clear')
nx = size(A,2);
nu = size(B,2);
x = sdpvar(nx,N+1);
u = sdpvar(nu,N);

feas = false;

state = [x(:,1) == x0];

if isempty(Af)
    for i = (N-5):(N+1)
        state = [state, x(:,i)==bf];
    end
else
    state = [state, Af*x(:,(N-5):(N+1))<=bf];
end

cost = x(:,N+1)'*P*x(:,N+1);

for k = 1:N
    state = [state, xL <= x(:,k+1),x(:,k+1)<= xU,...
        -0.3 <= x(9,k+1) - x(9,k), x(9,k+1) - x(9,k) <= 0.3];
    
    input = [uL <= u(:,k),u(:,k) <= uU];
    
    dynamics = x(:,k+1) == A*x(:,k) + B*u(:,k);

    cost = cost + (x(:,k) - x(:,N+1))'*Q*(x(:,k) - x(:,N+1)) + (u(:,k))'*R*(u(:,k));
end

options = sdpsettings('verbose',0,'solver','ipopt');
sol = optimize([state dynamics input],cost,options);

% assign(x,0);
% check(state)
% check(input)
% check(dynamics)

if sol.problem == 0
    feas = true;
else
    xOpt = [];
    uOpt = [];
    return;
end

xOpt = double(x);
uOpt = double(u);
