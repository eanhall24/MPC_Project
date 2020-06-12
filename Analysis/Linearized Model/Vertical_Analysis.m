%% Vertical Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

%% [0,0,1]

load('MPC_posx.mat')
load('MPC_posy.mat')
load('MPC_posz.mat')
load('MPC_time.mat')
load('MPC_teltime.mat')
load('MPC_mot1.mat')
load('MPC_mot2.mat')
load('MPC_mot3.mat')
load('MPC_mot4.mat')
MPCposx = posx;
MPCposy = posy;
MPCposz = posz;
MPCtime = time;
MPCteltime = teltime;
MPCmot1 = mot1;
MPCmot2 = mot2;
MPCmot3 = mot3;
MPCmot4 = mot4;

load('Rates_posx.mat')
load('Rates_posy.mat')
load('Rates_posz.mat')
load('Rates_time.mat')
load('Rates_teltime.mat')
load('Rates_mot1.mat')
load('Rates_mot2.mat')
load('Rates_mot3.mat')
load('Rates_mot4.mat')

MPCTindex = 1;

for i = 1:length(MPCposz)
    if MPCposz(i) > 0
        MPCTindex = i-1;
        break;
    end
end

MPCtime = MPCtime - MPCtime(MPCTindex);
desPos = [0;0;1];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = MPCTindex:length(MPCposz)
    if MPCposz(i) < lower(3) || MPCposz(i) > upper(3)
        ss_index = i;
    end
end

settlingTime = MPCtime(ss_index);
mpcsettlingTime = settlingTime;

MPCtelTindex = floor(MPCTindex/2);
MPCteltime = MPCteltime - MPCteltime(MPCtelTindex);

mpcMax = max(MPCmot1);
disp('The max MPC force is:')
disp(mpcMax)

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

figure('Name','Vertical Position')
% subplot(2,1,1)
plot(MPCtime(MPCTindex:end),MPCposz(MPCTindex:end),'b')
hold on
plot(MPCtime(MPCTindex:end),MPCposz(MPCTindex:end),'r--')
% plot(MPCtime(MPCTindex:end),MPCposy(MPCTindex:end),'g')
% plot(MPCtime(MPCTindex:end),MPCposz(MPCTindex:end),'r')
xline(mpcsettlingTime,'k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 1.1])
xlabel('Time(s)')
ylabel('Z Position(m)')
legend('MPC','UMPC','settling time')
title('Reference at [0,0,1]')

% subplot(2,1,2)
% plot(MPCtime(MPCTindex:end),MPCposx(MPCTindex:end),'b')
% hold on
% plot(MPCtime(MPCTindex:end),MPCposy(MPCTindex:end),'g')
% plot(MPCtime(MPCTindex:end),MPCposz(MPCTindex:end),'r')
% xline(mpcsettlingTime,'k');
% yline(lower(3),'--k');
% yline(upper(3),'--k');
% ylim([0 1.1])
% xlabel('Time(s)')
% ylabel('Position(m)')
% legend('posx','posy','posz','settling time')
% title('Unconstrained MPC: Target [0,0,1]')

figure('Name','Vertical Motor Forces')
% subplot(2,1,1)
plot(MPCteltime(MPCtelTindex:end),MPCmot1(MPCtelTindex:end))
hold on
plot(MPCteltime(MPCtelTindex:end),MPCmot2(MPCtelTindex:end))
plot(MPCteltime(MPCtelTindex:end),MPCmot3(MPCtelTindex:end))
plot(MPCteltime(MPCtelTindex:end),MPCmot4(MPCtelTindex:end))
ylabel('Motor Force(N)')
ylim([1.5 2.5])
legend('motor 1','motor 2','motor 3','motor 4')
title('Reference at [0,0,1]')

% subplot(2,1,2)
% plot(MPCteltime(MPCtelTindex:end),MPCmot1(MPCtelTindex:end))
% hold on
% plot(MPCteltime(MPCtelTindex:end),MPCmot2(MPCtelTindex:end))
% plot(MPCteltime(MPCtelTindex:end),MPCmot3(MPCtelTindex:end))
% plot(MPCteltime(MPCtelTindex:end),MPCmot4(MPCtelTindex:end))
% ylabel('Motor Force(N)')
% ylim([1.5 2.5])
% legend('motor 1','motor 2','motor 3','motor 4')
% title('Unconstrained MPC: Target [0,0,1]')

%% Vertical Position [0,0,10]

load('Large_MPC_posx.mat')
load('Large_MPC_posy.mat')
load('Large_MPC_posz.mat')
load('Large_MPC_time.mat')
load('Large_MPC_teltime.mat')
load('Large_MPC_mot1.mat')
load('Large_MPC_mot2.mat')
load('Large_MPC_mot3.mat')
load('Large_MPC_mot4.mat')
MPCposx = posx;
MPCposy = posy;
MPCposz = posz;
MPCtime = time;
MPCteltime = teltime;
MPCmot1 = mot1;
MPCmot2 = mot2;
MPCmot3 = mot3;
MPCmot4 = mot4;

load('Large_Rates_posx.mat')
load('Large_Rates_posy.mat')
load('Large_Rates_posz.mat')
load('Large_Rates_time.mat')
load('Large_Rates_teltime.mat')
load('Large_Rates_mot1.mat')
load('Large_Rates_mot2.mat')
load('Large_Rates_mot3.mat')
load('Large_Rates_mot4.mat')

MPCTindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        MPCTindex = i-1;
        break;
    end
end

MPCtime = MPCtime - MPCtime(MPCTindex);
desPos = [0;0;10];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = MPCTindex:length(posz)
    if posz(i) < lower(3) || posz(i) > upper(3)
        ss_index = i;
    end
end

settlingTime = MPCtime(ss_index);
mpcsettlingTime = settlingTime;

MPCtelTindex = floor(MPCTindex/2);
MPCteltime = MPCteltime - MPCteltime(MPCtelTindex);

mpcMax = max(MPCmot1);
disp('The max MPC force is:')
disp(mpcMax)

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
rates_settlingTime = settlingTime;

telTindex = floor(Tindex/2);
teltime = teltime - teltime(telTindex);

ratesMax = max(mot1);
disp('The max rates force is:')
disp(ratesMax)

figure('Name','Vertical Position')
subplot(2,1,1)
plot(time(MPCTindex:end),posx(MPCTindex:end),'b')
hold on
plot(time(MPCTindex:end),posy(MPCTindex:end),'g')
plot(time(MPCTindex:end),posz(MPCTindex:end),'r')
xline(mpcsettlingTime,'k');
yline(lower(3),'--k');
yline(upper(3),'--k');
ylim([0 11])
xlabel('Time(s)')
ylabel('Position(m)')
legend('posx','posy','posz','settling time')
title('MPC: Target [0,0,10]')

subplot(2,1,2)
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

figure('Name','Vertical Motor Forces')
subplot(2,1,1)
plot(teltime(MPCtelTindex:end),mot1(MPCtelTindex:end))
hold on
plot(teltime(MPCtelTindex:end),mot2(MPCtelTindex:end))
plot(teltime(MPCtelTindex:end),mot3(MPCtelTindex:end))
plot(teltime(MPCtelTindex:end),mot4(MPCtelTindex:end))
ylabel('Motor Force(N)')
ylim([1 4])
legend('motor 1','motor 2','motor 3','motor 4')
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
title('Rates: Target [0,0,10]')