%% Landing Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

load('Landing_MPC_posx.mat')
load('Landing_MPC_posz.mat')
MPCposx = posx;
MPCposz = posz;
load('Landing_Rates_posx.mat')
load('Landing_Rates_posz.mat')

figure('Name', 'Landing Analysis')
subplot(2,1,1)
plot(MPCposx, MPCposz)
ylabel('Z Position (m)')
title('MPC Controller')

subplot(2,1,2)
plot(posx, posz)
xlabel('X Position (m)')
ylabel('Z Position (m)')
title('Rates Controller')

