%% Braking Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

%% Horizontal Braking

load('Braking_MPC_posx.mat')
load('Braking_MPC_posz.mat')
load('Braking_MPC_time.mat')
MPCposx = posx;
MPCposz = posz;
MPCtime = time;
load('Braking_Rates_posx.mat')
load('Braking_Rates_posz.mat')
load('Braking_Rates_time.mat')

refMPC = 0.215;
refRates = 0.566;

penMPC = ((max(MPCposx) - refMPC)/refMPC)*100;
penRates = ((max(posx) - refRates)/refRates)*100;

figure('Name', 'Horizontal Penetration Analysis')
subplot(2,1,1)
plot(MPCtime, MPCposx)
hold on
yline(refMPC);
% xlim([0 1.2])
% ylim([0.8 1.2])
ylabel('Z Position (m)')
title('MPC Controller')

subplot(2,1,2)
plot(time, posx)
hold on
yline(refRates);
% xlim([0 1.2])
% ylim([0.8 1.2])
xlabel('X Position (m)')
ylabel('Z Position (m)')
title('Rates Controller')

%% Vertical Brakings

load('Vertical_MPC_posx.mat')
load('Vertical_MPC_posz.mat')
load('Vertical_MPC_time.mat')
MPCposx = posx;
MPCposz = posz;
MPCtime = time;
load('Vertical_Rates_posx.mat')
load('Vertical_Rates_posz.mat')
load('Vertical_Rates_time.mat')

refMPC = 1.545;
refRates = 1.648;

penMPC = ((max(MPCposz) - refMPC)/refMPC)*100;
penRates = ((max(posz) - refRates)/refRates)*100;

figure('Name', 'Vertical Penetration Analysis')
subplot(2,1,1)
plot(MPCtime, MPCposz)
hold on
yline(refMPC);
% xlim([0 1.2])
% ylim([0.8 1.2])
ylabel('Z Position (m)')
title('MPC Controller')

subplot(2,1,2)
plot(time, posz)
hold on
yline(refRates);
% xlim([0 1.2])
% ylim([0.8 1.2])
xlabel('X Position (m)')
ylabel('Z Position (m)')
title('Rates Controller')

