%% Vertical Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

%% MPC @ [0,0,1]

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
disp('The max MPC force is:')
disp(mpcMax)

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

%% Rates @ [0,0,1]

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

ratesMax = max(mot1);
disp('The max rates force is:')
disp(ratesMax)

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

%% Large MPC @ [0,0,10]

load('Large_MPC_posx.mat')
load('Large_MPC_posy.mat')
load('Large_MPC_posz.mat')
load('Large_MPC_time.mat')
load('Large_MPC_teltime.mat')
load('Large_MPC_mot1.mat')
load('Large_MPC_mot2.mat')
load('Large_MPC_mot3.mat')
load('Large_MPC_mot4.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);
desPos = [0;0;10];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posz)
    if posz(i) < lower(3) || posz(i) > upper(3)
        ss_index = i;
    end
end

settlingTime = time(ss_index);
largempcsettlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);

largempcMax = max(mot1);
disp('The max MPC force is:')
disp(largempcMax)

figure('Name','Large MPC')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 11])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('MPC: Target [0,0,10]')

subplot(2,1,2)
plot(teltime(telTindex:end),mot1(telTindex:end))
hold on
plot(teltime(telTindex:end),mot2(telTindex:end))
plot(teltime(telTindex:end),mot3(telTindex:end))
plot(teltime(telTindex:end),mot4(telTindex:end))
xlabel('Time(s)')
ylabel('Motor Force(N)')
ylim([1 4])
legend('motor 1','motor 2','motor 3','motor 4')

%% Rates @ [0,0,10]

load('Large_Rates_posx.mat')
load('Large_Rates_posy.mat')
load('Large_Rates_posz.mat')
load('Large_Rates_time.mat')
load('Large_Rates_teltime.mat')
load('Large_Rates_mot1.mat')
load('Large_Rates_mot2.mat')
load('Large_Rates_mot3.mat')
load('Large_Rates_mot4.mat')

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

time = time - time(Tindex);
desPos = [0;0;10];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posz)
    if posz(i) < lower(3) || posz(i) > upper(3)
        ss_index = i;
    end
end

settlingTime = time(ss_index);
largerates_settlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);

largeratesMax = max(mot1);
disp('The max rates force is:')
disp(largeratesMax)

figure('Name','Large Rates')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posy(Tindex:end),'g')
plot(time(Tindex:end),posz(Tindex:end),'r')
xline(settlingTime,'k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 11])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('Rates: Target [0,0,10]')

subplot(2,1,2)
plot(teltime(telTindex:end),mot1(telTindex:end))
hold on
plot(teltime(telTindex:end),mot2(telTindex:end))
plot(teltime(telTindex:end),mot3(telTindex:end))
plot(teltime(telTindex:end),mot4(telTindex:end))
xlabel('Time(s)')
ylabel('Motor Force(N)')
ylim([1 4])
legend('motor 1','motor 2','motor 3','motor 4')



