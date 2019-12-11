%% MPC Project
close all
clear
clc

%% Dynamics
% Load Dynamics
Dynamics;

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
N = 20;
n = size(A,2);

%% State and Input Constraints
uL = [0;0;0;0;g];
uU = [mass*g;mass*g;mass*g;mass*g;g];
xL = [-5;-20;-pi/6;-10000;-5;-20;-pi/6;-10000;0;-20;-pi;-10000];
xU = [5;20;pi/6;10000;5;20;pi/6;10000;5;20;pi;10000];

%% Objective Function
% stage cost x'Qx+u'Ru
Q = diag([1 0 0 0 1 0 0 0 1 0 0 0]);
R = eye(5);

%% MPC Design
% The following lines of code implement an MPC where the terminal set is
% equal to the origin
P = Q;
xN = [2;0;0;0;0;0;0;0;2;0;0;0];
bf = xN;

%% Simulation Setup
M = 25;
x0 = zeros(12,1);
xOpt = zeros(n,M+1);
xOpt(:,1) = x0;
uOpt = zeros(5,M);
xPred = zeros(n,N+1,M);
feas = false([1,M]);
predErr = zeros(n,M-N+1);
% [xOpt, uOpt, feas] = solver(A,B,P,Q,R,N,x0,xL,xU,uL,uU,xN,[]);

%% Simulation
figure('Name','Trajectory')
for t = 1:M
    fprintf('Solving simstep: %i\n',t)
    
    [x, u, feas(t)] = solver(A,B,P,Q,R,N,xOpt(:,t),xL,xU,uL,uU,bf,[]);
    
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
    xOpt(:,t+1) = x(:,2);
    
    % Plot Open Loop
    plot3(xOpt(1,:),xOpt(5,:),xOpt(9,:),'r--')
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

xTest = zeros(n,M+1);
xTest(:,1) = x0;

for i = 1:M
   xTest(:,i+1) = A*xTest(:,i) + B*uOpt(:,i); 
end