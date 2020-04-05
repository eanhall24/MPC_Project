%% Data Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

close all 
clear
clc

load('posz.mat')
load('time.mat')

predict_x = [0 0.00130976 0.00523898 0.0117876 0.0209557 0.0327432...
    0.0471501 0.0641764 0.0838221 0.106087 0.130972 0.158476 0.188599...
    0.221342 0.256705 0.294686 0.335287 0.378508 0.42301 0.467209 0.510863];

dt = 0.1;

timespan = 0:dt:2;

[m,n] = size(predict_x);

time = time - time(1,1669);

figure('Name','Position Comparison')
plot(time(1,1669:2070),posz(1,1669:2070))
hold on
plot(timespan,predict_x,'r--')
grid on

