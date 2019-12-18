%% MPC Project
close all
clear
clc

%% Dynamics
% Load Dynamics
Dynamics;

%% Set parameters
wind_disturbance = 1;
model_mismatch = 0;
dist_reject = 0;
stoc_dist_reject = 0;

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
Q = 10*diag([1 1 0 0 1 1 0 0 1 1 0 0]);
% Q = 5*diag([3 1 1 1 3 1 1 1 10 3 0 0]);
Q = Q + diag(ones(12,1));
R = .9*eye(nu);
% R = 0.01*eye(nu);

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
% f_wind = @(z) normrnd(0.1, 0.05);
f_wind = @(z) 0.08*z;
% f_wind = @(z) 0.15;

M = 150;%40;
x0 = [3;0;0;0;0;0;0;0;3;0;0;0];
h = x0(9);
r = 3;%x0(1);
traj_follow = 1;

% Controller specific parameters
xOpt_MPC = zeros(n,M+1);
xOpt_MPC(:,1) = x0;
xOpt_DR1 = zeros(n,M+1);
xOpt_DR1(:,1) = x0;
xOpt_DR2 = zeros(n,M+1);
xOpt_DR2(:,1) = x0;

uOpt_MPC = zeros(5,M);
uOpt_DR1 = zeros(5,M);
uOpt_DR2 = zeros(5,M);

xPred_MPC = zeros(n,N+1,M);
xPred_DR1 = zeros(n,N+1,M);
xPred_DR2 = zeros(n,N+1,M);

feas_MPC = false([1,M]);
feas_DR1 = false([1,M]);
feas_DR2 = false([1,M]);

dist_DR1 = 0;
dist_DR2 = 0;

MPC_continue = true ; 
DR1_continue = true ; 
DR2_continue = true ; 
% predErr = zeros(n,M-N+1);
% [xOpt, uOpt, feas] = solver(A,B,P,Q,R,N,x0,xL,xU,uL,uU,xN,[]);



% Create initial model of disturbance
heights = [0:0.1:5]; 
disturb.mu = 0 ;
disturb.stddev = 0 ; 
disturb.model = containers.Map(heights, zeros(1, length(heights))) ; 
disturb.bin = containers.Map("KeyType", "double", "ValueType", "any") ; 

% data_pts_needed = 8 ; 
iters_til_update = 2;%25 %10;% 5 ;

dist_bin = [];
update_dist = false ; 
eps = 0.0001 ; 
    
%% Simulation for Regular MPC
figure('Name','Trajectory')
for t = 1:M
    fprintf('Solving simstep: %i\n',t)
    if MPC_continue
        [x_MPC, u_MPC, feas_MPC(t)] = solver(A,B,P,Q,R,N,xOpt_MPC(:,t),xL,xU,uL,uU,bf,Af,Xd,uRef,t,h,r,traj_follow,0);
        if ~feas_MPC(t)
            warning('MPC problem infeasible--exiting simulation for MPC')
%             xOpt_MPC = [];
%             uOpt_MPC = [];
            MPC_continue = false;
        else
            % Save open loop predictions
            xPred_MPC(:,:,t) = x_MPC;

            % Save first optimal input of sequence
            uOpt_MPC(:,t) = u_MPC(:,1);
        end
    end
    
    if DR1_continue
        [x_DR1, u_DR1, feas_DR1(t)] = solver(A,B,P,Q,R,N,xOpt_DR1(:,t),xL,xU,uL,uU,bf,Af,Xd,uRef,t,h,r,traj_follow,dist_DR1);
        if ~feas_DR1(t)
            warning('MPC problem infeasible--exiting simulation for DR1')
%             xOpt_DR1 = [];
%             uOpt_DR1 = [];
            DR1_continue = false;
        else
            % Save open loop predictions
            xPred_DR1(:,:,t) = x_DR1;

            % Save first optimal input of sequence
            uOpt_DR1(:,t) = u_DR1(:,1);
        end
    end
    
    if DR2_continue
        [x_DR2, u_DR2, feas_DR2(t)] = solver(A,B,P,Q,R,N,xOpt_DR2(:,t),xL,xU,uL,uU,bf,Af,Xd,uRef,t,h,r,traj_follow,dist_DR2);
        if ~feas_DR2(t)
            warning('MPC problem infeasible--exiting simulation for DR2')
%             xOpt_DR2 = [];
%             uOpt_DR2 = [];
            DR2_continue = false;
        else
            % Save open loop predictions
            xPred_DR2(:,:,t) = x_DR2;

            % Save first optimal input of sequence
            uOpt_DR2(:,t) = u_DR2(:,1);
        end
    end 
    
    % Get disturbance
    if stoc_dist_reject
        dReal = f_wind(0) ; 
    else
        if MPC_continue
            z_state_MPC = xOpt_MPC(9,t) ; 
            dReal = f_wind(z_state_MPC) ;
        end
        
        if DR1_continue
            z_state_DR1 = xOpt_DR1(9,t) ; 
            dReal = f_wind(z_state_DR1) ;
        end
        
        if DR2_continue
            z_state_DR2 = xOpt_DR2(9,t) ; 
            dReal = f_wind(z_state_DR2) ; 
        end
        
    end


    % Compute next step of closed-loop trajectory
    if MPC_continue 
        if ~wind_disturbance && ~model_mismatch
            xOpt_MPC(:,t+1) = x_MPC(:,2);
        elseif wind_disturbance && ~model_mismatch
            xOpt_MPC(:,t+1) = A*xOpt_MPC(:,t) + B*uOpt_MPC(:,t) + G*dReal;
        elseif model_mismatch && ~wind_disturbance
            xOpt_MPC(:,t+1) = A_tilda*xOpt_MPC(:,t) + B_tilda*uOpt_MPC(:,t);
        else
            xOpt_MPC(:,t+1) = A_tilda*xOpt_MPC(:,t) + B_tilda*uOpt_MPC(:,t) + G*dReal;
        end
    end
    
    if DR1_continue
        if ~wind_disturbance && ~model_mismatch
            xOpt_DR1(:,t+1) = x_DR1(:,2);
        elseif wind_disturbance && ~model_mismatch
            xOpt_DR1(:,t+1) = A*xOpt_DR1(:,t) + B*uOpt_DR1(:,t) + G*dReal;
        elseif model_mismatch && ~wind_disturbance
            xOpt_DR1(:,t+1) = A_tilda*xOpt_DR1(:,t) + B_tilda*uOpt_DR1(:,t);
        else
            xOpt_DR1(:,t+1) = A_tilda*xOpt_DR1(:,t) + B_tilda*uOpt_DR1(:,t) + G*dReal;
        end
    end
    
    if DR2_continue
        if ~wind_disturbance && ~model_mismatch
            xOpt_DR2(:,t+1) = x_DR2(:,2);
        elseif wind_disturbance && ~model_mismatch
            xOpt_DR2(:,t+1) = A*xOpt_DR2(:,t) + B*uOpt_DR2(:,t) + G*dReal;
        elseif model_mismatch && ~wind_disturbance
            xOpt_DR2(:,t+1) = A_tilda*xOpt_DR2(:,t) + B_tilda*uOpt_DR2(:,t);
        else
            xOpt_DR2(:,t+1) = A_tilda*xOpt_DR2(:,t) + B_tilda*uOpt_DR2(:,t) + G*dReal;
        end
    end
    
    % Check if we need to update the disturbance model on this iter.
    update_dist = t >= iters_til_update; 
    % update_dist = mod(t,iters_til_update) == 0;
    

    % Compute disturbances for the next step
    if DR1_continue
        dist_DR1 = xOpt_DR1(:,t+1) - A*xOpt_DR1(:,t)-B*uOpt_DR1(:,t)
    end
    
    if DR2_continue
        % Measure disturbance. Only store a scalar var -> d. Not vector G*d
        Gd = xOpt_DR2(:,t+1) - A*xOpt_DR2(:,t)-B*uOpt_DR2(:,t);
        d = Gd(abs(Gd)>=eps) ; 
        disturb = updateBin(disturb, d, z_state_DR2) ; 
        
        % Update disturbance model if time
        if update_dist
            disturb = updateDisturbModel(disturb, dist_bin);            
        end
        
        % Sample from disturbance model to get disturbance
        val = sampleDist(disturb, z_state_DR2) ; 
        if isnan(val)
            dist_DR2 = xOpt_DR2(:,t+1) - A*xOpt_DR2(:,t)-B*uOpt_DR2(:,t)
        else
            dist_DR2 = G*sampleDist(disturb, z_state_DR2) 
        end
    end

    % Plot next
    plot3(x_MPC(1,:),x_MPC(5,:),x_MPC(9,:),'r--')
    plot3(x_DR1(1,:),x_DR1(5,:),x_DR1(9,:),'b--')
    plot3(x_DR2(1,:),x_DR2(5,:),x_DR2(9,:),'g--')
    legend("No disturbance Rejection = red", "w/ disturbance rejection method 1 = blue", "w/ disturbance rejection method 2 = green")
    grid on
    hold on
    % pause(0.01)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot Closed Loop

% Trajectory
plot3(xOpt_MPC(1,:),xOpt_MPC(5,:),xOpt_MPC(9,:),'ro-')
plot3(xOpt_DR1(1,:),xOpt_DR1(5,:),xOpt_DR1(9,:),'bo-')
plot3(xOpt_DR2(1,:),xOpt_DR2(5,:),xOpt_DR2(9,:),'co-')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal;
hold on

t_step = .1;
t_vec = t_step*((1:M+1) -1);
[x_des, ~] = path_gen(t_vec, 0, 5, r, h);
plot3(x_des(1,:),x_des(5,:),x_des(9,:),'go-')

figure
hold on
plot3(xOpt_MPC(1,:),xOpt_MPC(5,:),xOpt_MPC(9,:),'bo-')
plot3(xOpt_DR1(1,:),xOpt_DR1(5,:),xOpt_DR1(9,:),'go-')
plot3(xOpt_DR2(1,:),xOpt_DR2(5,:),xOpt_DR2(9,:),'co-')
plot3(x_des(1,:),x_des(5,:),x_des(9,:),'ro-')
title('Trajectory Following with Disturbance Rejection')
legend("MPC", "Disturbance Rejection Method 1", "Disturbance Rejection Method 2", 'Desired Trajectory')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal
grid on
hold off

%%
figure 
hold on
plot3(xOpt_MPC(1,:),xOpt_MPC(5,:),xOpt_MPC(9,:),'ro-')
plot3(xOpt_DR1(1,:),xOpt_DR1(5,:),xOpt_DR1(9,:),'bo-')
plot3(xOpt_DR2(1,:),xOpt_DR2(5,:),xOpt_DR2(9,:),'co-')
legend("MPC", "Disturbance Rejection Method 1", "Disturbance Rejection Method 2")
hold off
%% Position
figure('Name','Position')
subplot(3,1,1)
plot(xOpt_MPC(1,:))
plot(xOpt_DR1(1,:))
plot(xOpt_DR2(1,:))
title('X position')
ylabel('position(m)')
legend("MPC", "Disturbance Rejection Method 1", "Disturbance Rejection Method 2")
subplot(3,1,2)
plot(xOpt_MPC(5,:))
plot(xOpt_DR1(5,:))
plot(xOpt_DR2(5,:))
title('Y position')
ylabel('position(m)')
legend("MPC", "Disturbance Rejection Method 1", "Disturbance Rejection Method 2")
subplot(3,1,3)
plot(xOpt_MPC(9,:))
plot(xOpt_DR1(9,:))
plot(xOpt_DR2(9,:))
title('Z position')
ylabel('position(m)')
legend("MPC", "Disturbance Rejection Method 1", "Disturbance Rejection Method 2")
xlabel('timestep(k)')


%% Velocity
figure('Name','Velocity')
subplot(3,1,1)
plot(xOpt_MPC(2,:),'r')
plot(xOpt_DR1(2,:),'b')
plot(xOpt_DR2(2,:),'g')
title('X Velocity')
ylabel('velocity(m/s)')
subplot(3,1,2)
plot(xOpt_MPC(6,:),'r')
plot(xOpt_DR1(6,:),'b')
plot(xOpt_DR2(6,:),'g')
title('Y Velocity')
ylabel('velocity(m/s)')
subplot(3,1,3)
plot(xOpt_MPC(10,:),'r')
plot(xOpt_DR1(10,:),'b')
plot(xOpt_DR2(10,:),'g')
title('Z Velocity')
ylabel('velocity(m/s)')
xlabel('timestep(k)')

%%
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
% x0 = zeros(12,1);
% xk = x0;
% xOpt_2 = zeros(n,M+1);
% xOpt_2(:,1) = x0;
% uOpt_2 = zeros(5,M);
% for k =1:M
%     uk = [-K*(xk-Xd);g];
%     if ~wind_disturbance && ~model_mismatch
%         xk1 = A*xk + B*uk;
%     elseif wind_disturbance && ~model_mismatch
%         xk1 = A*xk + B*uk + G*f_wind(xk(9));
%     elseif ~wind_disturbance && model_mismatch
%         xk1 = A_tilda*xk + B_tilda*uk;
%     else
%         xk1 = A_tilda*xk + B_tilda*uk + G*f_wind(xk(9));
%     end
%     uOpt_2(:,k) = uk;
%     xOpt_2(:,k+1) = xk1;
%     xk = xk1;  
% end

% % Trajectory
% figure;
% plot3(xOpt_2(1,:),xOpt_2(5,:),xOpt_2(9,:),'bo-')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% axis equal;
% grid on;

% % Position
% figure('Name','Position LQR')
% subplot(3,1,1)
% plot(xOpt_2(1,:))
% title('X position')
% ylabel('position(m)')
% subplot(3,1,2)
% plot(xOpt_2(5,:))
% title('Y position')
% ylabel('position(m)')
% subplot(3,1,3)
% plot(xOpt_2(9,:))
% title('Z position')
% ylabel('position(m)')
% xlabel('timestep(k)')

% % Velocity
% figure('Name','Velocity LQR')
% subplot(3,1,1)
% plot(xOpt_2(2,:),'r')
% title('X Velocity')
% ylabel('velocity(m/s)')
% subplot(3,1,2)
% plot(xOpt_2(6,:),'r')
% title('Y Velocity')
% ylabel('velocity(m/s)')
% subplot(3,1,3)
% plot(xOpt_2(10,:),'r')
% title('Z Velocity')
% ylabel('velocity(m/s)')
% xlabel('timestep(k)')

% % Orientation
% figure('Name','Orientation LQR')
% subplot(3,1,1)
% plot(xOpt_2(3,:),'k')
% title('Pitch')
% ylabel('pitch(rad)')
% subplot(3,1,2)
% plot(xOpt_2(7,:),'k')
% title('Roll')
% ylabel('roll(rad)')
% subplot(3,1,3)
% plot(xOpt_2(11,:),'k')
% title('Yaw')
% ylabel('Yaw(rad')
% xlabel('timestep(k)')

% % Motor Forces
% figure('Name','Motor Forces LQR')
% plot(uOpt_2(1,:))
% hold on
% plot(uOpt_2(2,:))
% plot(uOpt_2(3,:))
% plot(uOpt_2(4,:))
% title('Motor Forces')
% xlabel('timestep(k)')
% ylabel('Force(N)')
% legend('Motor 1','Motor 2','Motor 3','Motor 4')


function disturb = updateDisturbModel(disturb, bin)
    % Calculate mu and standard deviation for the gaussian distribution
%     disturb.mu = mean(bin);  
%     total = 0 ; 
%     for i=1:length(bin)
%         total = total + (bin(i)-disturb.mu)^2 ; 
%     end
%     disturb.stddev = sqrt(total/length(bin));
    keySet = keys(disturb.model) ; 
    for i=1:length(keySet)
        key = keySet{i} ; 
        if disturb.bin.isKey(key)
            disturb.model(key) = mean(disturb.bin(key)) ; 
        end
    end
end

function new_dist = sampleDist(disturb, z_state)
    % new_dist = normrnd(disturb.mu, disturb.stddev)
    % new_dist = disturb.mu 
    key = stateToKey(z_state) ; 
    if ~disturb.model.isKey(key)
        disturb.model(key) = NaN;
    end
    new_dist = disturb.model(key) ; 
end

function disturb = updateBin(disturb, d, z_state)
    key = stateToKey(z_state) ; 
    if ~disturb.bin.isKey(key)
        disturb.bin(key) = [] ; 
    end
    disturb.bin(key) = [disturb.bin(key) d] ; 
end

function key = stateToKey(state)
    heights = [0:0.05:5];
    [~,closestIndex] = min(abs(heights-state)) ; 
    key = heights(closestIndex) ; 
end