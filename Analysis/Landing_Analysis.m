%% Landing Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

load('Landing_MPC_posx.mat')
load('Landing_MPC_posz.mat')
load('Landing_MPC_time.mat')
load('Landing_MPC_pitch.mat')
load('Landing_MPC_velz.mat')
load('Landing_MPC_esttime.mat')
MPCposx = posx;
MPCposz = posz;
MPCtime = time;
MPCpitch = pitch;
MPCvelz = velz;
MPCesttime = esttime;
load('Landing_Rates_posx.mat')
load('Landing_Rates_posz.mat')
load('Landing_Rates_time.mat')
load('Landing_Rates_pitch.mat')
load('Landing_Rates_velz.mat')
load('Landing_Rates_esttime.mat')

figure('Name', 'Landing Analysis')
% subplot(2,1,1)
plot(MPCposx, MPCposz,'b')
hold on
plot(posx, posz,'b--')
legend('MPC','PD')
xlabel('X Position (m)')
ylabel('Z Position (m)')
title('Position')

MPCtime = MPCtime - MPCtime(1635);
time = time - time(1184);
MPCesttime = MPCesttime - MPCesttime(33);
esttime = esttime - esttime(36);

maxMPC = (360/(2*pi))*max(MPCpitch);
maxR = (360/(2*pi))*max(pitch);
disp(maxMPC)
disp(maxR)

figure('Name', 'Orientation Analysis')
% subplot(2,1,1)
plot(MPCtime(1635:end), (360/(2*pi))*MPCpitch(1635:end),'r')
hold on
plot(time(1184:end), (360/(2*pi))*pitch(1184:end),'r--')
legend('MPC','PD')
xlabel('time (s)')
ylabel('Pitch Angle (degrees)')
title('Orientation')

figure('Name', 'Velocity Analysis')
% subplot(2,1,1)
plot(MPCesttime(33:end), MPCvelz(33:end),'r')
hold on
plot(esttime(36:end), velz(36:end),'r--')
legend('MPC','PD')
xlabel('time (s)')
ylabel('Pitch Angle (degrees)')
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