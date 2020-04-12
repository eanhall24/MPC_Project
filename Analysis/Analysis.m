%% Data Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

%% MPC @ [0,0,1]
% close all
% clear
% clc
% 
% load('MPC_posx.mat')
% load('MPC_posy.mat')
% load('MPC_posz.mat')
% load('MPC_time.mat')
% 
% Tindex = 1;
% 
% for i = 1:length(posz)
%     if posz(i) > 0
%         Tindex = i-1;
%         break;
%     end
% end
% 
% time = time - time(Tindex);
% desPos = [0;0;1];
% lower = desPos - 0.02*desPos;
% upper = desPos + 0.02*desPos;
% ss_index = 1;
% 
% for i = Tindex:length(posz)
%     if posz(i) < lower(3) || posz(i) > upper(3)
%         ss_index = i;
%     end
% end
% 
% settlingTime = time(ss_index);
% 
% figure('Name','MPC')
% plot(time(Tindex:end),posx(Tindex:end),'b')
% hold on
% plot(time(Tindex:end),posy(Tindex:end),'g')
% plot(time(Tindex:end),posz(Tindex:end),'r')
% xline(settlingTime,'k');
% yline(lower(3),'--k');
% yline(upper(3),'--k');
% ylim([0 1.1])
% xlabel('Time(s)')
% ylabel('Position(m)')
% legend('posx','posy','posz','settling time')
% title('MPC: Target [0,0,1]')
% 
% disp('The settling time for the MPC controller is:')
% disp(settlingTime)

%% MPC @ [0,0,1] with same gains as PD
close all
clear
clc

load('MPC3_posx.mat')
load('MPC3_posy.mat')
load('MPC3_posz.mat')
load('MPC3_time.mat')
load('MPC3_teltime.mat')
load('MPC3_mot1.mat')
load('MPC3_mot2.mat')
load('MPC3_mot3.mat')
load('MPC3_mot4.mat')

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

for i = 1:length(mot1)
    if mot1(i) > 0.1
        telTindex = i-1;
        break;
    end
end

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

for i = 1:length(mot1)
    if mot1(i) > 0.1
        telTindex = i-1;
        break;
    end
end

teltime = teltime - teltime(telTindex);

ratesMax = max(mot1);

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
legend('motor 1','motor 2','motor 3','motor 4')

disp('The settling time for the MPC controller is:')
disp(settlingTime)

%% MPC @ [1,2,2]
clc

load('MPC1_posx.mat')
load('MPC1_posy.mat')
load('MPC1_posz.mat')
load('MPC1_time.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);
desPos = [1;2;2];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posz)
    if (posx(i) < lower(1) || posx(i) > upper(1)) ||...
            (posy(i) < lower(2) || posy(i) > upper(2)) ||...
            (posz(i) < lower(3) || posz(i) > upper(3))
        ss_index = i;
    end
end

settlingTime = time(ss_index);

figure('Name','MPC_')
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(1),'--k');
yline(upper(1),'--k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 2.1])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('MPC: Target [1,2,2]')

disp('The settling time for the MPC controller is:')
disp(settlingTime)

%% Rates @ [1,2,2]
clc

load('Rates1_posx.mat')
load('Rates1_posy.mat')
load('Rates1_posz.mat')
load('Rates1_time.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);
desPos = [1;2;2];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posz)
    if (posx(i) < lower(1) || posx(i) > upper(1)) ||...
            (posy(i) < lower(2) || posy(i) > upper(2)) ||...
            (posz(i) < lower(3) || posz(i) > upper(3))
        ss_index = i;
    end
end

settlingTime = time(ss_index);

figure('Name','Rates_')
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(1),'--k');
yline(upper(1),'--k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 2.1])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('Rates: Target [1,2,2]')

disp('The settling time for the MPC controller is:')
disp(settlingTime)

