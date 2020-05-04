%% Landing Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

load('Landing_MPC_posx.mat')
load('Landing_MPC_posz.mat')
load('Landing_MPC_time.mat')
MPCposx = posx;
MPCposz = posz;
MPCtime = time;
load('Landing_Rates_posx.mat')
load('Landing_Rates_posz.mat')
load('Landing_Rates_posz.mat')

figure('Name', 'Landing Analysis')
% subplot(2,1,1)
plot(MPCposx, MPCposz,'b')
hold on
plot(posx, posz,'b--')
legend('MPC','PD')
xlabel('X Position (m)')
ylabel('Z Position (m)')
title('Landing')

% subplot(2,1,2)
% plot(posx, posz)
% xlabel('X Position (m)')
% ylabel('Z Position (m)')
% title('Rates Controller')

ref1 = [2;0];
Q1 = 0.1*diag([20 16]);
Ucost = 0;
Upose = [MPCposx;MPCposz];
Ccost = 0;
pose = [posx;posz];

%% Unconstrained Cost
for i = 2869:3304
    Ucost = Ucost + (ref1-Upose(i))'*Q1*(ref1-Upose(i));
end

%% Constrained Cost
for i = 2408:2805
    Ccost = Ccost + (ref1-pose(i))'*Q1*(ref1-pose(i));
end

disp(Ucost)
disp(Ccost)

time1 = MPCtime(3304)-MPCtime(2869)
time2 = time(2805)-time(2408)