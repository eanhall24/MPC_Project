%% Timestep Analysis
% Timestep of 0.1 seconds with different time horizons.
% Goal position is [1,0,1] meters

close all
clear
clc

load('timestep1/lin1_posx.mat')
load('timestep1/lin1_posy.mat')
load('timestep1/lin1_posz.mat')
load('timestep1/lin1_pitch.mat')
load('timestep1/lin1_mot1.mat')
load('timestep1/lin1_mot2.mat')
load('timestep1/lin1_mot3.mat')
load('timestep1/lin1_mot4.mat')
load('timestep1/lin1_time.mat')
load('timestep1/lin1_teltime.mat')

posx1 = posx;
posz1 = posz;
time1 = time;
pitch1 = pitch;
mot11 = mot1;
mot21 = mot2;
mot31 = mot3;
mot41 = mot4;
teltime1 = teltime;

load('timestep1/nonlin1_posx.mat')
load('timestep1/nonlin1_posy.mat')
load('timestep1/nonlin1_posz.mat')
load('timestep1/nonlin1_pitch.mat')
load('timestep1/nonlin1_mot1.mat')
load('timestep1/nonlin1_mot2.mat')
load('timestep1/nonlin1_mot3.mat')
load('timestep1/nonlin1_mot4.mat')
load('timestep1/nonlin1_time.mat')
load('timestep1/nonlin1_teltime.mat')

posx2 = posx;
posz2 = posz;
time2 = time;
pitch2 = pitch;
mot12 = mot1;
mot22 = mot2;
mot32 = mot3;
mot42 = mot4;
teltime2 = teltime;

load('timestep1/Forward_UMPC1_posx.mat')
load('timestep1/Forward_UMPC1_posy.mat')
load('timestep1/Forward_UMPC1_posz.mat')
load('timestep1/Forward_UMPC1_pitch.mat')
load('timestep1/Forward_UMPC1_mot1.mat')
load('timestep1/Forward_UMPC1_mot2.mat')
load('timestep1/Forward_UMPC1_mot3.mat')
load('timestep1/Forward_UMPC1_mot4.mat')
load('timestep1/Forward_UMPC1_time.mat')
load('timestep1/Forward_UMPC1_teltime.mat')

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
yline(1,'--k');
xlim([0 15])
ylim([0 1.5])
ylabel('Position(m)')
legend('linear posx','linear posz','nonlinear posx','nonlinear posz','location','best')
title('Position')

subplot(2,1,2)
plot(time1(Tindex1:end),(360/(2*pi))*pitch1(Tindex1:end),'b')
hold on
plot(time2(Tindex2:end),(360/(2*pi))*pitch2(Tindex2:end),'r')
yline(0,'--k');
xlim([0 15])
% ylim([0 0.6])
ylabel('Pitch Angle (deg)')
legend('linear pitch','nonlinear pitch','location','best')
title('Pitch')

figure('Name','Motor Forces')
subplot(2,1,1)
plot(teltime1(telindex1:end),mot11(telindex1:end))
hold on
plot(teltime1(telindex1:end),mot21(telindex1:end))
plot(teltime1(telindex1:end),mot31(telindex1:end))
plot(teltime1(telindex1:end),mot41(telindex1:end))
xlim([0 8])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Linear Model Motor Forces')

subplot(2,1,2)
plot(teltime2(telindex2:end),mot12(telindex2:end))
hold on
plot(teltime2(telindex2:end),mot22(telindex2:end))
plot(teltime2(telindex2:end),mot32(telindex2:end))
plot(teltime2(telindex2:end),mot42(telindex2:end))
xlim([0 8])
ylim([1.5 1.9])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Nonlinear Model Motor Forces')

figure('Name','Pose Data')
subplot(2,1,1)
plot(time(Tindex:end),posx(Tindex:end),'b')
hold on
plot(time(Tindex:end),posz(Tindex:end),'r')
yline(1,'--k');
xlim([0 15])
ylim([0 1.5])
ylabel('Position(m)')
legend('posx','posz','location','best')
title('Unconstrained Nonlinear Position')

subplot(2,1,2)
plot(time(Tindex:end),(360/(2*pi))*pitch(Tindex:end),'b')
yline(0,'--k');
xlim([0 15])
% ylim([0 0.6])
ylabel('Pitch Angle (deg)')
legend('linear pitch','nonlinear pitch','location','best')
title('Unconstrained Nonlinear Pitch')


