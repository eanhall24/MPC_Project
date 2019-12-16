%% MPC Project
close all
clear
clc

%% Dynamics
% Load Dynamics
Dynamics;
wind_disturbance = 1;
model_mismatch = 0;
dist_reject = 1;

%% Continuous System Test
% The quadcopter should slow down after each input and hover at the end.
% u = [0.25*mass*g*ones(4,1);g];
% u1 = [0.25*1.2*mass*g*ones(4,1);g];
% u2 = [0.25*0.8*mass*g*ones(4,1);g];
% u = repmat(u,1,51);
% u1 = repmat(u1,1,75);
% u2 = repmat(u2,1,75);
% u = [u1 u2 u];
% t = 0:0.1:20;
% x0 = [2;0;0;0;1;0;0;0;1;0;0;0];
% [y,t,x] = lsim(system,u,t,x0);

%% Discrete System Test
% The quadcopter should slow down after each input and hover at the end.
% u1 = [0.25*1.4*mass*g*ones(4,1);g];
% u2 = [0.25*0.6*mass*g*ones(4,1);g];
% u4 = [0.25*mass*g*ones(4,1);g];
% 
% N = 25;
% u1 = repmat(u1,1,10);
% u2 = repmat(u2,1,10);
% u4 = repmat(u4,1,5);
% u = [u1 u2 u4];
% x0 = zeros(12,1);
% x = zeros(12,N+1);
% x(:,1) = x0;
% 
% for i = 1:N
%     x(:,i+1) = A*x(:,i) + B*u(:,i);
% end
% 
% figure(1)
% plot3(x(1,:),x(5,:),x(9,:))
% xlim([0 1])
% ylim([0 1])
% 
% figure(2)
% plot(x(9,:))

%% MPC Horizon
N = 8;
n = size(A,2);
nu = 4;

%% State and Input Constraints
uL = [0;0;0;0;g];
% uU = [mass*g;mass*g;mass*g;mass*g;g];
uU = [14.7;14.7;14.7;14.7;g];
xL = [-5;-20;-pi/6;-10000;-5;-20;-pi/6;-10000;0;-20;-pi;-10000];
xU = [5;20;pi/6;10000;5;20;pi/6;10000;5;20;pi;10000];

X = Polyhedron('lb',xL,'ub',xU);
U = Polyhedron('lb',uL(1:nu),'ub',uU(1:nu));

%% Objective Function
% stage cost x'Qx+u'Ru
% Q = 10*diag([1 1 0 0 1 1 0 0 1 1 0 0]);
Q = 5*diag([3 1 1 1 3 1 1 1 10 3 0 0]);
Q = Q + diag(ones(12,1));
% R = eye(nu);
R = 0.01*eye(nu);

%% MPC Design
% % The following lines of code implement an MPC where the terminal set is
% % equal to xN
% P = Q;
% xN = [2;0;0;0;0;0;0;0;2;0;0;0];
% bf = xN;
% Af = [];

%%

% The following for bigger Xf (coming from LQR Oinf)
% Closed loop system of 2c
[K,P]=dlqr(A,B(:,1:nu),Q,R);
% closed loop system
Acl=A-B(:,1:nu)*K;
% % remeber to convet input constraits in state constraints
Xtilde=X.intersect(Polyhedron('H',[-U.H(:,1:nu)*K U.H(:,nu+1)])); 
Oinf=N_pos_inv(Acl,Xtilde);
% % figure(50);
% % plot(Oinf); title('Xf');

Af = Oinf.H(:,1:n);
bf = Oinf.H(:,n+1);

% Af = [];
% bf = [];

Xd = [3;0;0;0;0;0;0;0;3;0;0;0];

%% Simulation Setup
M = 100;%40;
% x0 = zeros(12,1);
x0 = [2;0;0;0;0;0;0;0;3;0;0;0];
h = x0(9);
r = 3;%x0(1);
traj_follow = 1;
xOpt = zeros(n,M+1);
xOpt(:,1) = x0;
uOpt = zeros(5,M);
xPred = zeros(n,N+1,M);
feas = false([1,M]);
predErr = zeros(n,M-N+1);
% [xOpt, uOpt, feas] = solver(A,B,P,Q,R,N,x0,xL,xU,uL,uU,xN,[]);

xk_1 = [];
uk_1 = [];

%% Simulation
figure('Name','Trajectory')
for t = 1:M
    fprintf('Solving simstep: %i\n',t)
    
    [x, u, feas(t)] = solver(A,B,P,Q,R,N,xOpt(:,t),xL,xU,uL,uU,bf,Af,Xd,uRef,t,h,r,traj_follow);
    
    if ~feas(t)
        warning('MPC problem infeasible--exiting simulation')
        xOpt = [];
        uOpt = [];
        return;
    end 
    
    % Save open loop predictions
    xPred(:,:,t) = x;
    
    % Save first optimal input of sequence
    uOpt(:,t) = u(:,1);
    % Compute next step of closed-loop trajectory
    if ~wind_disturbance && ~model_mismatch
        xOpt(:,t+1) = x(:,2);
    elseif wind_disturbance && ~model_mismatch
        xOpt(:,t+1) = A*xOpt(:,t) + B*uOpt(:,t) + G*f_wind(xOpt(9,t));
    elseif model_mismatch && ~wind_disturbance
        xOpt(:,t+1) = A_tilda*xOpt(:,t) + B_tilda*uOpt(:,t);
    else
        xOpt(:,t+1) = A_tilda*xOpt(:,t) + B_tilda*uOpt(:,t) + G*f_wind(xOpt(9,t));
    end
    
    if ~dist_reject
        xk_1 = [];
    else
        xk_1 = xOpt(:,t);
        uk_1 = uOpt(:,t);
    end
    
    % Plot Open Loop
    plot3(x(1,:),x(5,:),x(9,:),'r--')
    grid on
    hold on
    pause(0.1)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Closed Loop

% Trajectory
plot3(xOpt(1,:),xOpt(5,:),xOpt(9,:),'bo-')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal;

% Position
figure('Name','Position')
subplot(3,1,1)
plot(xOpt(1,:))
title('X position')
ylabel('position(m)')
subplot(3,1,2)
plot(xOpt(5,:))
title('Y position')
ylabel('position(m)')
subplot(3,1,3)
plot(xOpt(9,:))
title('Z position')
ylabel('position(m)')
xlabel('timestep(k)')

% Velocity
figure('Name','Velocity')
subplot(3,1,1)
plot(xOpt(2,:),'r')
title('X Velocity')
ylabel('velocity(m/s)')
subplot(3,1,2)
plot(xOpt(6,:),'r')
title('Y Velocity')
ylabel('velocity(m/s)')
subplot(3,1,3)
plot(xOpt(10,:),'r')
title('Z Velocity')
ylabel('velocity(m/s)')
xlabel('timestep(k)')

% Orientation
figure('Name','Orientation')
subplot(3,1,1)
plot(xOpt(3,:),'k')
title('Pitch')
ylabel('pitch(rad)')
subplot(3,1,2)
plot(xOpt(7,:),'k')
title('Roll')
ylabel('roll(rad)')
subplot(3,1,3)
plot(xOpt(11,:),'k')
title('Yaw')
ylabel('Yaw(rad')
xlabel('timestep(k)')

% Motor Forces
figure('Name','Motor Forces')
plot(uOpt(1,:))
hold on
plot(uOpt(2,:))
plot(uOpt(3,:))
plot(uOpt(4,:))
title('Motor Forces')
xlabel('timestep(k)')
ylabel('Force(N)')
legend('Motor 1','Motor 2','Motor 3','Motor 4')

% Find the prediction error
% for i = 1:length(predErr)
%     err = xOpt(:,i:i+N)-xPred(:,:,i);
%     predErr(:,i) = [norm(err(1,:)) norm(err(2,:))]';
% end

% xTest = zeros(n,M+1);
% xTest(:,1) = x0;
% 
% for i = 1:M
%    xTest(:,i+1) = A*xTest(:,i) + B*uOpt(:,i); 
% end

%% Compare with LQR
x0 = zeros(12,1);
xk = x0;
xOpt_2 = zeros(n,M+1);
xOpt_2(:,1) = x0;
uOpt_2 = zeros(5,M);
for k =1:M
    uk = [-K*(xk-Xd);g];
    if ~wind_disturbance && ~model_mismatch
        xk1 = A*xk + B*uk;
    elseif wind_disturbance && ~model_mismatch
        xk1 = A*xk + B*uk + G*f_wind(xk(9));
    elseif ~wind_disturbance && model_mismatch
        xk1 = A_tilda*xk + B_tilda*uk;
    else
        xk1 = A_tilda*xk + B_tilda*uk + G*f_wind(xk(9));
    end
    uOpt_2(:,k) = uk;
    xOpt_2(:,k+1) = xk1;
    xk = xk1;  
end

% Trajectory
figure;
plot3(xOpt_2(1,:),xOpt_2(5,:),xOpt_2(9,:),'bo-')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal;
grid on;

% Position
figure('Name','Position LQR')
subplot(3,1,1)
plot(xOpt_2(1,:))
title('X position')
ylabel('position(m)')
subplot(3,1,2)
plot(xOpt_2(5,:))
title('Y position')
ylabel('position(m)')
subplot(3,1,3)
plot(xOpt_2(9,:))
title('Z position')
ylabel('position(m)')
xlabel('timestep(k)')

% Velocity
figure('Name','Velocity LQR')
subplot(3,1,1)
plot(xOpt_2(2,:),'r')
title('X Velocity')
ylabel('velocity(m/s)')
subplot(3,1,2)
plot(xOpt_2(6,:),'r')
title('Y Velocity')
ylabel('velocity(m/s)')
subplot(3,1,3)
plot(xOpt_2(10,:),'r')
title('Z Velocity')
ylabel('velocity(m/s)')
xlabel('timestep(k)')

% Orientation
figure('Name','Orientation LQR')
subplot(3,1,1)
plot(xOpt_2(3,:),'k')
title('Pitch')
ylabel('pitch(rad)')
subplot(3,1,2)
plot(xOpt_2(7,:),'k')
title('Roll')
ylabel('roll(rad)')
subplot(3,1,3)
plot(xOpt_2(11,:),'k')
title('Yaw')
ylabel('Yaw(rad')
xlabel('timestep(k)')

% Motor Forces
figure('Name','Motor Forces LQR')
plot(uOpt_2(1,:))
hold on
plot(uOpt_2(2,:))
plot(uOpt_2(3,:))
plot(uOpt_2(4,:))
title('Motor Forces')
xlabel('timestep(k)')
ylabel('Force(N)')
legend('Motor 1','Motor 2','Motor 3','Motor 4')


