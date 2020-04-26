%% Data Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

%% MPC @ [0,0,1] with same gains as PD
close all
clear
clc

load('MPC_posx.mat')
load('MPC_posy.mat')
load('MPC_posz.mat')
load('MPC_time.mat')
load('MPC_teltime.mat')
load('MPC_mot1.mat')
load('MPC_mot2.mat')
load('MPC_mot3.mat')
load('MPC_mot4.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);
desPos = [0;0;1];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posz)
    if posz(i) < lower(3) || posz(i) > upper(3)
        ss_index = i;
    end
end

settlingTime = time(ss_index);
mpcsettlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);

mpcMax = max(mot1);

figure('Name','MPC')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 1.1])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('MPC: Target [0,0,1]')

subplot(2,1,2)
plot(teltime(telTindex:end),mot1(telTindex:end))
hold on
plot(teltime(telTindex:end),mot2(telTindex:end))
plot(teltime(telTindex:end),mot3(telTindex:end))
plot(teltime(telTindex:end),mot4(telTindex:end))
xlabel('Time(s)')
ylabel('Motor Force(N)')
ylim([1.5 2.5])
legend('motor 1','motor 2','motor 3','motor 4')

disp('The settling time for the MPC controller is:')
disp(settlingTime)

%% Rates @ [0,0,1]
clc

load('Rates_posx.mat')
load('Rates_posy.mat')
load('Rates_posz.mat')
load('Rates_time.mat')
load('Rates_teltime.mat')
load('Rates_mot1.mat')
load('Rates_mot2.mat')
load('Rates_mot3.mat')
load('Rates_mot4.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);
desPos = [0;0;1];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posz)
    if posz(i) < lower(3) || posz(i) > upper(3)
        ss_index = i;
    end
end

settlingTime = time(ss_index);
rates_settlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);

figure('Name','Rates')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 1.1])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('Rates: Target [0,0,1]')

subplot(2,1,2)
plot(teltime(telTindex:end),mot1(telTindex:end))
hold on
plot(teltime(telTindex:end),mot2(telTindex:end))
plot(teltime(telTindex:end),mot3(telTindex:end))
plot(teltime(telTindex:end),mot4(telTindex:end))
xlabel('Time(s)')
ylabel('Motor Force(N)')
ylim([1.5 2.5])
legend('motor 1','motor 2','motor 3','motor 4')

disp('The settling time for the rates controller is:')
disp(settlingTime)

%% Rates Forward @ [2,0,1]
clc

load('Forward_Rates_posx.mat')
load('Forward_Rates_posy.mat')
load('Forward_Rates_posz.mat')
load('Forward_Rates_time.mat')
load('Forward_Rates_teltime.mat')
load('Forward_Rates_mot1.mat')
load('Forward_Rates_mot2.mat')
load('Forward_Rates_mot3.mat')
load('Forward_Rates_mot4.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);

for i = 1:length(time)
    if time(i) > 6
        Tindex = i;
        break;
    end
end

desPos = [2;0;0];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posx)
    if posx(i) < lower(1) || posx(i) > upper(1)
        ss_index = i;
    end
end

settlingTime = time(ss_index);
forward_rates_settlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);

[M,I] = max(mot3);

ratesinput = [mot1(I); mot2(I); mot3(I); mot4(I)];

figure('Name','Forward Rates')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(1),'--k');
yline(upper(1),'--k');
ylim([0 2.1])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('Rates: Target [2,0,1]')

subplot(2,1,2)
plot(teltime(telTindex:end),mot1(telTindex:end))
hold on
plot(teltime(telTindex:end),mot2(telTindex:end))
plot(teltime(telTindex:end),mot3(telTindex:end))
plot(teltime(telTindex:end),mot4(telTindex:end))
xlabel('Time(s)')
ylabel('Motor Force(N)')
% ylim([1.5 2.5])
legend('motor 1','motor 2','motor 3','motor 4')

disp('The settling time for the rates controller is:')
disp(settlingTime)

%% Unconstrained MPC Forward @ [2,0,1]
clc

load('Forward_UMPC_posx.mat')
load('Forward_UMPC_posy.mat')
load('Forward_UMPC_posz.mat')
load('Forward_UMPC_time.mat')
load('Forward_UMPC_teltime.mat')
load('Forward_UMPC_mot1.mat')
load('Forward_UMPC_mot2.mat')
load('Forward_UMPC_mot3.mat')
load('Forward_UMPC_mot4.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);

for i = 1:length(time)
    if time(i) > 6
        Tindex = i;
        break;
    end
end

desPos = [2;0;0];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posx)
    if posx(i) < lower(1) || posx(i) > upper(1)
        ss_index = i;
    end
end

settlingTime = time(ss_index);
forward_umpc_settlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);

[M,I] = max(mot3);

umpcinput = [mot1(I); mot2(I); mot3(I); mot4(I)];

figure('Name','Forward Unconstrained MPC')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(1),'--k');
yline(upper(1),'--k');
ylim([0 2.1])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('MPC: Target [2,0,1]')

subplot(2,1,2)
plot(teltime(telTindex:end),mot1(telTindex:end))
hold on
plot(teltime(telTindex:end),mot2(telTindex:end))
plot(teltime(telTindex:end),mot3(telTindex:end))
plot(teltime(telTindex:end),mot4(telTindex:end))
xlabel('Time(s)')
ylabel('Motor Force(N)')
% ylim([1.5 2.5])
legend('motor 1','motor 2','motor 3','motor 4')

disp('The settling time for the rates controller is:')
disp(settlingTime)

%% MPC Forward @ [2,0,1]
clc

load('Forward_MPC_posx.mat')
load('Forward_MPC_posy.mat')
load('Forward_MPC_posz.mat')
load('Forward_MPC_time.mat')
load('Forward_MPC_teltime.mat')
load('Forward_MPC_mot1.mat')
load('Forward_MPC_mot2.mat')
load('Forward_MPC_mot3.mat')
load('Forward_MPC_mot4.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);

for i = 1:length(time)
    if time(i) > 6
        Tindex = i;
        break;
    end
end

desPos = [2;0;0];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posx)
    if posx(i) < lower(1) || posx(i) > upper(1)
        ss_index = i;
    end
end

settlingTime = time(ss_index);
forward_mpc_settlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);


[M,I] = max(mot3);

mpcinput = [mot1(I); mot2(I); mot3(I); mot4(I)];

figure('Name','Forward MPC')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(1),'--k');
yline(upper(1),'--k');
ylim([0 2.1])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('MPC: Target [2,0,1]')

subplot(2,1,2)
plot(teltime(telTindex:end),mot1(telTindex:end))
hold on
plot(teltime(telTindex:end),mot2(telTindex:end))
plot(teltime(telTindex:end),mot3(telTindex:end))
plot(teltime(telTindex:end),mot4(telTindex:end))
xlabel('Time(s)')
ylabel('Motor Force(N)')
% ylim([1.5 2.5])
legend('motor 1','motor 2','motor 3','motor 4')

disp('The settling time for the rates controller is:')
disp(settlingTime)



