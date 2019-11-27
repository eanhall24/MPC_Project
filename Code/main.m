%% MPC Project
close all
clear
clc

%% Dynamics
% Load Dynamics
Dynamics;

%% Continuous System Test
% The quadcopter should slow down after each input and hover at the end.
u = [0.25*mass*g*ones(4,1);g];
u = repmat(u,1,101);
t = 0:0.1:10;
x0 = [2;1;2;0;0;0;0;0;0;0;0;0];
[y,t,x] = lsim(system,u,t,x0);

%% Discrete System Test
% The quadcopter should slow down after each input and hover at the end.
% u1 = [0.25*1.4*mass*g*ones(4,1);g];
% u2 = [0.25*1.2*mass*g*ones(4,1);g];
% u3 = [0.25*1.05*mass*g*ones(4,1);g];
% u4 = [0.25*mass*g*ones(4,1);g];
% 
% N = 10;
% u = [u1 u2 u3 u4 u4 u4 u4 u4 u4 u4];
% x0 = zeros(12,1);
% x = zeros(12,N+1);
% x(:,1) = x0;
% 
% for i = 1:N
%     x(:,i+1) = A*x(:,i) + B*u(:,i);
% end
% 
% figure(1)
% plot3(x(1,:),x(2,:),x(3,:))
% xlim([0 1])
% ylim([0 1])
% 
% figure(2)
% plot(x(3,:))

%% MPC
% N = 3;
% M = 10;
% x0 = zeros(12,1);
% n = size(x0,1);
% 
% feas = false([1,M]);
% xOpt = zeros(n,M+1);
% uOpt = zeros(4,M);
% predErr = zeros(n,M-N+1);
% 
% uL = [0;-0.05;-0.05;-0.05];
% uU = [2*mass*g;0.05;0.05;0.05];
% xL = [-3;-3;0;0;0;-2;-pi/6;-pi/6;-pi;-pi/12;-pi/12;-pi/2];
% xU = [3;3;3;1.5;1.5;2;pi/6;pi/6;pi;pi/12;pi/12;pi/2];
% Q = diag([1 1 1 0 0 0 0 0 0 0 0 0]);
% P = Q;
% R = 1;
% 
% xOpt(:,1) = x0;
% xPred = zeros(n,N+1,M);
% 
% xN = [0;0;2;0;0;0;0;0;0;0;0;0];
% 
% % [xOpt, uOpt, feas] = solver(A,B,E,P,Q,R,N,x0,xL,xU,uL,uU,xN,[]);
% 
% figure
% for t = 1:M
%     fprintf('Solving simstep: %i\n',t)
%     
%     [x, u, feas(t)] = solver(A,B,E,P,Q,R,N,xOpt(:,t),xL,xU,uL,uU,xN,[]);
%     
%     if ~feas(t)
%         xOpt = [];
%         uOpt = [];
%         predErr = [];
%         return;
%     end
%     
%     % Save open loop predictions
%     xPred(:,:,t) = x;
%     
%     % Save closed loop trajectory
%     xOpt(:,t+1) = x(:,2);
%     uOpt(:,t) = u(:,1);
%     
%     % Plot Open Loop
%     plot3(x(1,:),x(2,:),x(3,:),'r--')
%     grid on
%     hold on
%     pause(0.1)
% end

% Plot Closed Loop
% plot(xOpt(1,:),xOpt(2,:),'bo-')
% Find the prediction error
% for i = 1:length(predErr)
%     err = xOpt(:,i:i+N)-xPred(:,:,i);
%     predErr(:,i) = [norm(err(1,:)) norm(err(2,:))]';
% end