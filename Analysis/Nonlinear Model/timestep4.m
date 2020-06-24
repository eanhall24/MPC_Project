%% Timestep Analysis
% Timestep of 0.02 seconds with different time horizons.
% Goal position is [1,0,1] meters

close all
clear
clc

load('timestep4/Forward_MPC4_posx.mat')
load('timestep4/Forward_MPC4_posy.mat')
load('timestep4/Forward_MPC4_posz.mat')
load('timestep4/Forward_MPC4_pitch.mat')
load('timestep4/Forward_MPC4_mot1.mat')
load('timestep4/Forward_MPC4_mot2.mat')
load('timestep4/Forward_MPC4_mot3.mat')
load('timestep4/Forward_MPC4_mot4.mat')
load('timestep4/Forward_MPC4_time.mat')
load('timestep4/Forward_MPC4_teltime.mat')

posx1 = posx;
posz1 = posz;
time1 = time;
pitch1 = pitch;
mot11 = mot1;
mot21 = mot2;
mot31 = mot3;
mot41 = mot4;
teltime1 = teltime;

load('timestep4/Forward_MPC402_posx.mat')
load('timestep4/Forward_MPC402_posy.mat')
load('timestep4/Forward_MPC402_posz.mat')
load('timestep4/Forward_MPC402_pitch.mat')
load('timestep4/Forward_MPC402_mot1.mat')
load('timestep4/Forward_MPC402_mot2.mat')
load('timestep4/Forward_MPC402_mot3.mat')
load('timestep4/Forward_MPC402_mot4.mat')
load('timestep4/Forward_MPC402_time.mat')
load('timestep4/Forward_MPC402_teltime.mat')

posx2 = posx;
posz2 = posz;
time2 = time;
pitch2 = pitch;
mot12 = mot1;
mot22 = mot2;
mot32 = mot3;
mot42 = mot4;
teltime2 = teltime;

load('timestep4/Forward_MPC416_posx.mat')
load('timestep4/Forward_MPC416_posy.mat')
load('timestep4/Forward_MPC416_posz.mat')
load('timestep4/Forward_MPC416_pitch.mat')
load('timestep4/Forward_MPC416_mot1.mat')
load('timestep4/Forward_MPC416_mot2.mat')
load('timestep4/Forward_MPC416_mot3.mat')
load('timestep4/Forward_MPC416_mot4.mat')
load('timestep4/Forward_MPC416_time.mat')
load('timestep4/Forward_MPC416_teltime.mat')

%% Data analysis
Tindex1 = 1;
for i = 1:length(posz1)
    if posz1(i) > 0
        Tindex1 = i-1;
        break;
    end
end

for i = 1:length(teltime1)
    if teltime1(i) > time1(Tindex1)
        telindex1 = i;
        break;
    end
end

teltime1 = teltime1 - time1(Tindex1);
time1 = time1 - time1(Tindex1);

Tindex2 = 1;
for i = 1:length(posz2)
    if posz2(i) > 0
        Tindex2 = i-1;
        break;
    end
end

for i = 1:length(teltime2)
    if teltime2(i) > time2(Tindex2)
        telindex2 = i;
        break;
    end
end

teltime2 = teltime2 - time2(Tindex2);
time2 = time2 - time2(Tindex2);

Tindex = 1;
for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

for i = 1:length(teltime)
    if teltime(i) > time(Tindex)
        telindex = i;
        break;
    end
end

teltime = teltime - time(Tindex);
time = time - time(Tindex);

%% Plots

desPos = [1;0;0];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;

figure('Name','Pose Data')
subplot(2,1,1)
plot(time1(Tindex1:end),posx1(Tindex1:end),'b')
hold on
plot(time1(Tindex1:end),posz1(Tindex1:end),'b--')
plot(time2(Tindex2:end),posx2(Tindex2:end),'r')
plot(time2(Tindex2:end),posz2(Tindex2:end),'r--')
plot(time(Tindex:end),posx(Tindex:end),'k')
plot(time(Tindex:end),posz(Tindex:end),'k--')
yline(1,'--g');
xlim([0 15])
ylim([0 1.5])
ylabel('Position(m)')
legend('0.8 posx','0.8 posz','0.2 posx','0.2 posz','1.6 posx',...
    '1.6 posz','location','best')
title('Position')

subplot(2,1,2)
plot(time1(Tindex1:end),(360/(2*pi))*pitch1(Tindex1:end),'b')
hold on
plot(time2(Tindex2:end),(360/(2*pi))*pitch2(Tindex2:end),'r')
plot(time(Tindex:end),(360/(2*pi))*pitch(Tindex:end),'k')
yline(0,'--g');
xlim([0 15])
% ylim([0 0.6])
ylabel('Pitch Angle (deg)')
legend('0.8 pitch','0.2 pitch','1.6 pitch','location','best')
title('Pitch')

figure('Name','Motor Forces')
subplot(3,1,1)
plot(teltime1(telindex1:end),mot11(telindex1:end))
hold on
plot(teltime1(telindex1:end),mot21(telindex1:end))
plot(teltime1(telindex1:end),mot31(telindex1:end))
plot(teltime1(telindex1:end),mot41(telindex1:end))
xlim([0 8])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Time Horizon of 0.8 seconds')

subplot(3,1,2)
plot(teltime2(telindex2:end),mot12(telindex2:end))
hold on
plot(teltime2(telindex2:end),mot22(telindex2:end))
plot(teltime2(telindex2:end),mot32(telindex2:end))
plot(teltime2(telindex2:end),mot42(telindex2:end))
xlim([0 8])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Time Horizon of 0.2 seconds')

subplot(3,1,3)
plot(teltime(telindex:end),mot1(telindex:end))
hold on
plot(teltime(telindex:end),mot2(telindex:end))
plot(teltime(telindex:end),mot3(telindex:end))
plot(teltime(telindex:end),mot4(telindex:end))
xlim([0 8])
xlabel('Time(s)')
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Time Horizon of 1.6 seconds')


