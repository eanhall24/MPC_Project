%% MPC Project
close all
clear
clc

% load('Pose1.mat')
% load('Motors1.mat')
% load('Pose2.mat')
% load('Motors2.mat')
% load('Pose3.mat')
% load('Motors3.mat')

%% Dynamics
% Load Dynamics
Dynamics;
wind_disturbance = 0;
model_mismatch = 0;
dist_reject = 0;



%% MPC Horizon
N = 3;
n = size(A,2);
nu = 4;

%% State and Input Constraints
% uL = [0.025*mass*g;0.025*mass*g;0.025*mass*g;0.025*mass*g;g];
uL = [zeros(4,1);g];
uU = [14.7;14.7;14.7;14.7;g];
xL = [-5;-20;-pi/6;-10000;-5;-20;-pi/6;-10000;0;-20;-pi;-10000];
xU = [5;20;pi/6;10000;5;20;pi/6;10000;5;20;pi;10000];

X = Polyhedron('lb',xL,'ub',xU);
U = Polyhedron('lb',uL(1:nu),'ub',uU(1:nu));

%% Objective Function
% stage cost x'Qx+u'Ru
Q = 5*diag([3 1 1 1 3 1 1 1 10 3 0.5 0]);
Q = Q + diag(ones(12,1));
% R = 1*eye(nu);
R = 0.01*eye(nu);
% Q = diag(ones(12,1));
% R = eye(nu);

%% MPC Design
%% The following lines of code implement an MPC where the terminal set is
% % equal to xN
% P = Q;
% xN = [2;0;0;0;2;0;0;0;2;0;0;0];
% bf = xN;
% Af = [];

%% The following for bigger Xf (coming from LQR Oinf)
% Closed loop system of 2c
[K,P]=dlqr(A,B(:,1:nu),Q,R);
% closed loop system
Acl=A-B(:,1:nu)*K;
% remeber to convet input constraits in state constraints
Xtilde=X.intersect(Polyhedron('H',[-U.H(:,1:nu)*K U.H(:,nu+1)])); 
Oinf=N_pos_inv(Acl,Xtilde);
% figure(50);
% plot(Oinf); title('Xf');

Af = Oinf.H(:,1:n);
bf = Oinf.H(:,n+1);

Xd = [2;0;0;0;2;0;0;0;2;0;0;0];
% Xd = [-1;0;0;0;3.5;0;0;0;4;0;0;0];
% Xd = [3;0;0;0;1;0;0;0;2;0;pi/4;0];

%% Simulation Setup
M = 50;
x0 = zeros(12,1);
xOpt = zeros(n,M+1);
xOpt(:,1) = x0;
uOpt = zeros(5,M);
xPred = zeros(n,N+1,M);
feas = false([1,M]);
predErr = zeros(n,M-N+1);
% [xOpt, uOpt, feas] = solver(A,B,P,Q,R,N,x0,xL,xU,uL,uU,xN,[]);

xk_1 = [];
uk_1 = [];

%% LQR
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

%% Simulation
figure('Name','Trajectory')
for t = 1:M
    fprintf('Solving simstep: %i\n',t)
    
    [x, u, feas(t)] = solver(A,B,P,Q,R,N,xOpt(:,t),xL,xU,uL,uU,bf,Af,Xd,uRef,G,xk_1,uk_1);
    
    if ~feas(t)
        fprintf('MPC problem infeasible--exiting simulation')
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
    xlim([-5 5])
    ylim([-5 5])
    zlim([0 5])
    grid on
    hold on
    pause(0.1)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Closed Loop

time = 0:0.1:M*0.1;
load('Pose1.mat')
load('Motors1.mat')
% load('Pose2.mat')
% load('Motors2.mat')
% load('Pose3.mat')
% load('Motors3.mat')

% Trajectory
plot3(xOpt(1,:),xOpt(5,:),xOpt(9,:),'bo-')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal;

% Trajectory
figure('Name','Trajectory')
plot3(xOpt(1,:),xOpt(5,:),xOpt(9,:),'bo-')
hold on
plot3(posX(350:end),posY(350:end),posZ(350:end),'ko-')
plot3(xOpt_2(1,:),xOpt_2(5,:),xOpt_2(9,:),'go-')
legend('MPC','PD Controller','LQR')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal;

% Position
figure('Name','Position')
subplot(3,1,1)
plot(time,xOpt(1,:))
hold on
plot(time,xOpt_2(1,:))
plot(simTime(1:end-349)-simTime(1),posX(350:end))
title('X position')
ylabel('position(m)')
legend('MPC','LQR','PD Controller')
xlim([0 5])
subplot(3,1,2)
plot(time,xOpt(5,:))
hold on
plot(time,xOpt_2(5,:))
plot(simTime(1:end-349)-simTime(1),posY(350:end))
title('Y position')
ylabel('position(m)')
xlim([0 5])
subplot(3,1,3)
plot(time,xOpt(9,:))
hold on
plot(time,xOpt_2(9,:))
plot(simTime(1:end-349)-simTime(1),posZ(350:end))
title('Z position')
ylabel('position(m)')
xlim([0 5])
xlabel('time(s)')

% Velocity
figure('Name','Velocity')
subplot(3,1,1)
plot(time,xOpt(2,:))
hold on
plot(time,xOpt_2(2,:))
plot(simTime(1:end-349)-simTime(1),velX(350:end))
title('X Velocity')
legend('MPC','LQR','PD Controller')
xlim([0 5])
ylabel('velocity(m/s)')
subplot(3,1,2)
plot(time,xOpt(6,:))
hold on
plot(time,xOpt_2(6,:))
plot(simTime(1:end-349)-simTime(1),velY(350:end))
title('Y Velocity')
xlim([0 5])
ylabel('velocity(m/s)')
subplot(3,1,3)
plot(time,xOpt(10,:))
hold on
plot(time,xOpt_2(10,:))
plot(simTime(1:end-349)-simTime(1),velZ(350:end))
title('Z Velocity')
xlim([0 5])
ylabel('velocity(m/s)')
xlabel('time(s)')

% Orientation
figure('Name','Orientation')
subplot(3,1,1)
plot(time,xOpt(3,:))
hold on
plot(time,xOpt_2(3,:))
plot(simTime(1:end-349)-simTime(1),pitch(350:end))
title('Pitch')
legend('MPC','LQR','PD Controller')
xlim([0 5])
ylabel('pitch(rad)')
subplot(3,1,2)
plot(time,xOpt(7,:))
hold on
plot(time,xOpt_2(7,:))
plot(simTime(1:end-349)-simTime(1),roll(350:end))
title('Roll')
xlim([0 5])
ylabel('roll(rad)')
subplot(3,1,3)
plot(time,xOpt(11,:))
hold on
plot(time,xOpt_2(11,:))
plot(simTime(1:end-349)-simTime(1),yaw(350:end))
title('Yaw')
ylabel('Yaw(rad')
xlim([0 5])
xlabel('time(s)')

% Motor Forces
motor0 = smooth(motor0);
motor1 = smooth(motor1);
motor2 = smooth(motor2);
motor3 = smooth(motor3);
figure('Name','Motor Forces')
subplot(3,1,1)
plot(time(1:end-1),uOpt(1,:))
hold on
plot(time(1:end-1),uOpt(2,:))
plot(time(1:end-1),uOpt(3,:))
plot(time(1:end-1),uOpt(4,:))
title('MPC')
ylabel('Force(N)')
xlim([0 5])
legend('Motor 1','Motor 2','Motor 3','Motor 4')
subplot(3,1,2)
plot(time(1:end-1),uOpt_2(1,:))
hold on
plot(time(1:end-1),uOpt_2(2,:))
plot(time(1:end-1),uOpt_2(3,:))
plot(time(1:end-1),uOpt_2(4,:))
title('LQR')
ylabel('Force(N)')
xlim([0 5])
legend('Motor 1','Motor 2','Motor 3','Motor 4')
subplot(3,1,3)
plot(telTime(1:end-174)-telTime(1),motor0(175:end))
hold on
plot(telTime(1:end-174)-telTime(1),motor1(175:end))
plot(telTime(1:end-174)-telTime(1),motor2(175:end))
plot(telTime(1:end-174)-telTime(1),motor3(175:end))
title('PD')
xlabel('time(s)')
ylabel('Force(N)')
xlim([0 5])
legend('Motor 1','Motor 2','Motor 3','Motor 4')

%% Plot ROS Simulation
% load('Pose1.mat')
% load('Motors1.mat')
% figure('Name','ROS Trajectory')
% plot3(posX(350:end),posY(350:end),posZ(350:end),'ro-')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% axis equal;

% Position
% figure('Name','ROS Position')
% subplot(3,1,1)
% plot(simTime(1:end-349)-simTime(1),posX(350:end))
% title('X position')
% ylabel('position(m)')
% subplot(3,1,2)
% plot(simTime(1:end-349)-simTime(1),posY(350:end))
% title('Y position')
% ylabel('position(m)')
% subplot(3,1,3)
% plot(simTime(1:end-349)-simTime(1),posZ(350:end))
% title('Z position')
% ylabel('position(m)')
% xlabel('time(s)')

% Velocity
% figure('Name','ROS Velocity')
% subplot(3,1,1)
% plot(simTime(1:end-349)-simTime(1),velX(350:end),'r')
% title('X Velocity')
% ylabel('velocity(m/s)')
% subplot(3,1,2)
% plot(simTime(1:end-349)-simTime(1),velY(350:end),'r')
% title('Y Velocity')
% ylabel('velocity(m/s)')
% subplot(3,1,3)
% plot(simTime(1:end-349)-simTime(1),velZ(350:end),'r')
% title('Z Velocity')
% ylabel('velocity(m/s)')
% xlabel('time(s)')

% Orientation
% figure('Name','ROS Orientation')
% subplot(3,1,1)
% plot(simTime(1:end-349)-simTime(1),pitch(350:end))
% title('Pitch')
% ylabel('pitch(rad)')
% subplot(3,1,2)
% plot(simTime(1:end-349)-simTime(1),roll(350:end))
% title('Roll')
% ylabel('roll(rad)')
% subplot(3,1,3)
% plot(simTime(1:end-349)-simTime(1),yaw(350:end))
% title('Yaw')
% ylabel('Yaw(rad')
% xlabel('time(s)')

% motor0 = smooth(motor0);
% motor1 = smooth(motor1);
% motor2 = smooth(motor2);
% motor3 = smooth(motor3);
% Motor Forces
% figure('Name','ROS Motor Forces')
% plot(telTime(1:end-174)-telTime(1),motor0(175:end))
% hold on
% plot(telTime(1:end-174)-telTime(1),motor1(175:end))
% plot(telTime(1:end-174)-telTime(1),motor2(175:end))
% plot(telTime(1:end-174)-telTime(1),motor3(175:end))
% title('Motor Forces')
% xlabel('time(s)')
% ylabel('Force(N)')
% legend('Motor 1','Motor 2','Motor 3','Motor 4')

% load('Pose2.mat')
% load('Motors2.mat')
% load('Pose3.mat')
% load('Motors3.mat')


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
 