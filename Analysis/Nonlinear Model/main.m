%% Timestep Analysis
% Timestep of 0.1 seconds with different time horizons.
% Goal position is [1,0,1] meters

close all
clear
clc

%% Load
load('lqr_posx.mat')
load('lqr_posy.mat')
load('lqr_posz.mat')
load('lqr_pitch.mat')
load('lqr_mot1.mat')
load('lqr_mot2.mat')
load('lqr_mot3.mat')
load('lqr_mot4.mat')
load('lqr_time.mat')
load('lqr_teltime.mat')

lqrposx = posx;
lqrposy = posy;
lqrposz = posz;
lqrpitch = pitch;
lqrtime = time;
mot1lqr = mot1;
mot2lqr = mot2;
mot3lqr = mot3;
mot4lqr = mot4;
lqrteltime = teltime;

load('timestep2/Test1_posx.mat')
load('timestep2/Test1_posy.mat')
load('timestep2/Test1_posz.mat')
load('timestep2/Test1_pitch.mat')
load('timestep2/Test1_mot1.mat')
load('timestep2/Test1_mot2.mat')
load('timestep2/Test1_mot3.mat')
load('timestep2/Test1_mot4.mat')
load('timestep2/Test1_time.mat')
load('timestep2/Test1_teltime.mat')

posx1 = posx;
posz1 = posz;
time1 = time;
pitch1 = pitch;
mot11 = mot1;
mot21 = mot2;
mot31 = mot3;
mot41 = mot4;
teltime1 = teltime;

load('timestep2/Test2_posx.mat')
load('timestep2/Test2_posy.mat')
load('timestep2/Test2_posz.mat')
load('timestep2/Test2_pitch.mat')
load('timestep2/Test2_mot1.mat')
load('timestep2/Test2_mot2.mat')
load('timestep2/Test2_mot3.mat')
load('timestep2/Test2_mot4.mat')
load('timestep2/Test2_time.mat')
load('timestep2/Test2_teltime.mat')

posx2 = posx;
posz2 = posz;
time2 = time;
pitch2 = pitch;
mot12 = mot1;
mot22 = mot2;
mot32 = mot3;
mot42 = mot4;
teltime2 = teltime;

load('timestep2/Test7_posx.mat')
load('timestep2/Test7_posy.mat')
load('timestep2/Test7_posz.mat')
load('timestep2/Test7_pitch.mat')
load('timestep2/Test7_mot1.mat')
load('timestep2/Test7_mot2.mat')
load('timestep2/Test7_mot3.mat')
load('timestep2/Test7_mot4.mat')
load('timestep2/Test7_time.mat')
load('timestep2/Test7_teltime.mat')

posx7 = posx;
posz7 = posz;
time7 = time;
pitch7 = pitch;
mot17 = mot1;
mot27 = mot2;
mot37 = mot3;
mot47 = mot4;
teltime7 = teltime;

load('timestep2/Test8_posx.mat')
load('timestep2/Test8_posy.mat')
load('timestep2/Test8_posz.mat')
load('timestep2/Test8_pitch.mat')
load('timestep2/Test8_mot1.mat')
load('timestep2/Test8_mot2.mat')
load('timestep2/Test8_mot3.mat')
load('timestep2/Test8_mot4.mat')
load('timestep2/Test8_time.mat')
load('timestep2/Test8_teltime.mat')

posx8 = posx;
posz8 = posz;
time8 = time;
pitch8 = pitch;
mot18 = mot1;
mot28 = mot2;
mot38 = mot3;
mot48 = mot4;
teltime8 = teltime;

load('timestep4/Test3_posx.mat')
load('timestep4/Test3_posy.mat')
load('timestep4/Test3_posz.mat')
load('timestep4/Test3_pitch.mat')
load('timestep4/Test3_mot1.mat')
load('timestep4/Test3_mot2.mat')
load('timestep4/Test3_mot3.mat')
load('timestep4/Test3_mot4.mat')
load('timestep4/Test3_time.mat')
load('timestep4/Test3_teltime.mat')

posx3 = posx;
posz3 = posz;
time3 = time;
pitch3 = pitch;
mot13 = mot1;
mot23 = mot2;
mot33 = mot3;
mot43 = mot4;
teltime3 = teltime;

load('timestep4/Test4_posx.mat')
load('timestep4/Test4_posy.mat')
load('timestep4/Test4_posz.mat')
load('timestep4/Test4_pitch.mat')
load('timestep4/Test4_mot1.mat')
load('timestep4/Test4_mot2.mat')
load('timestep4/Test4_mot3.mat')
load('timestep4/Test4_mot4.mat')
load('timestep4/Test4_time.mat')
load('timestep4/Test4_teltime.mat')

posx4 = posx;
posz4 = posz;
time4 = time;
pitch4 = pitch;
mot14 = mot1;
mot24 = mot2;
mot34 = mot3;
mot44 = mot4;
teltime4 = teltime;

load('timestep4/Test9_posx.mat')
load('timestep4/Test9_posy.mat')
load('timestep4/Test9_posz.mat')
load('timestep4/Test9_pitch.mat')
load('timestep4/Test9_mot1.mat')
load('timestep4/Test9_mot2.mat')
load('timestep4/Test9_mot3.mat')
load('timestep4/Test9_mot4.mat')
load('timestep4/Test9_time.mat')
load('timestep4/Test9_teltime.mat')

posx9 = posx;
posz9 = posz;
time9 = time;
pitch9 = pitch;
mot19 = mot1;
mot29 = mot2;
mot39 = mot3;
mot49 = mot4;
teltime9 = teltime;

load('timestep4/Test10_posx.mat')
load('timestep4/Test10_posy.mat')
load('timestep4/Test10_posz.mat')
load('timestep4/Test10_pitch.mat')
load('timestep4/Test10_mot1.mat')
load('timestep4/Test10_mot2.mat')
load('timestep4/Test10_mot3.mat')
load('timestep4/Test10_mot4.mat')
load('timestep4/Test10_time.mat')
load('timestep4/Test10_teltime.mat')

posx10 = posx;
posz10 = posz;
time10 = time;
pitch10 = pitch;
mot110 = mot1;
mot210 = mot2;
mot310 = mot3;
mot410 = mot4;
teltime10 = teltime;

load('timestep1/Test5_posx.mat')
load('timestep1/Test5_posy.mat')
load('timestep1/Test5_posz.mat')
load('timestep1/Test5_pitch.mat')
load('timestep1/Test5_mot1.mat')
load('timestep1/Test5_mot2.mat')
load('timestep1/Test5_mot3.mat')
load('timestep1/Test5_mot4.mat')
load('timestep1/Test5_time.mat')
load('timestep1/Test5_teltime.mat')

posx5 = posx;
posz5 = posz;
time5 = time;
pitch5 = pitch;
mot15 = mot1;
mot25 = mot2;
mot35 = mot3;
mot45 = mot4;
teltime5 = teltime;

load('timestep1/Test6_posx.mat')
load('timestep1/Test6_posy.mat')
load('timestep1/Test6_posz.mat')
load('timestep1/Test6_pitch.mat')
load('timestep1/Test6_mot1.mat')
load('timestep1/Test6_mot2.mat')
load('timestep1/Test6_mot3.mat')
load('timestep1/Test6_mot4.mat')
load('timestep1/Test6_time.mat')
load('timestep1/Test6_teltime.mat')

posx6 = posx;
posz6 = posz;
time6 = time;
pitch6 = pitch;
mot16 = mot1;
mot26 = mot2;
mot36 = mot3;
mot46 = mot4;
teltime6 = teltime;

load('timestep1/Test11_posx.mat')
load('timestep1/Test11_posy.mat')
load('timestep1/Test11_posz.mat')
load('timestep1/Test11_pitch.mat')
load('timestep1/Test11_mot1.mat')
load('timestep1/Test11_mot2.mat')
load('timestep1/Test11_mot3.mat')
load('timestep1/Test11_mot4.mat')
load('timestep1/Test11_time.mat')
load('timestep1/Test11_teltime.mat')

posx11 = posx;
posz11 = posz;
time11 = time;
pitch11 = pitch;
mot111 = mot1;
mot211 = mot2;
mot311 = mot3;
mot411 = mot4;
teltime11 = teltime;

load('timestep1/Test12_posx.mat')
load('timestep1/Test12_posy.mat')
load('timestep1/Test12_posz.mat')
load('timestep1/Test12_pitch.mat')
load('timestep1/Test12_mot1.mat')
load('timestep1/Test12_mot2.mat')
load('timestep1/Test12_mot3.mat')
load('timestep1/Test12_mot4.mat')
load('timestep1/Test12_time.mat')
load('timestep1/Test12_teltime.mat')

posx12 = posx;
posz12 = posz;
time12 = time;
pitch12 = pitch;
mot112 = mot1;
mot212 = mot2;
mot312 = mot3;
mot412 = mot4;
teltime12 = teltime;

%% Data analysis
lqrTindex = 1;
for i = 1:length(lqrposz)
    if lqrposz(i) > 0
        lqrTindex = i-1;
        break;
    end
end

for i = 1:length(lqrteltime)
    if lqrteltime(i) > lqrtime(lqrTindex)
        lqrtelindex = i;
        break;
    end
end

lqrteltime = lqrteltime - lqrtime(lqrTindex);
lqrtime = lqrtime - lqrtime(lqrTindex);

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

Tindex3 = 1;
for i = 1:length(posz3)
    if posz3(i) > 0
        Tindex3 = i-1;
        break;
    end
end

for i = 1:length(teltime3)
    if teltime3(i) > time3(Tindex3)
        telindex3 = i;
        break;
    end
end

teltime3 = teltime3 - time3(Tindex3);
time3 = time3 - time3(Tindex3);

Tindex4 = 1;
for i = 1:length(posz4)
    if posz4(i) > 0
        Tindex4 = i-1;
        break;
    end
end

for i = 1:length(teltime4)
    if teltime4(i) > time4(Tindex4)
        telindex4 = i;
        break;
    end
end

teltime4 = teltime4 - time4(Tindex4);
time4 = time4 - time4(Tindex4);

Tindex5 = 1;
for i = 1:length(posz5)
    if posz5(i) > 0
        Tindex5 = i-1;
        break;
    end
end

for i = 1:length(teltime5)
    if teltime5(i) > time5(Tindex5)
        telindex5 = i;
        break;
    end
end

teltime5 = teltime5 - time5(Tindex5);
time5 = time5 - time5(Tindex5);

Tindex6 = 1;
for i = 1:length(posz6)
    if posz6(i) > 0
        Tindex6 = i-1;
        break;
    end
end

for i = 1:length(teltime6)
    if teltime6(i) > time6(Tindex6)
        telindex6 = i;
        break;
    end
end

teltime6 = teltime6 - time6(Tindex6);
time6 = time6 - time6(Tindex6);

Tindex7 = 1;
for i = 1:length(posz7)
    if posz7(i) > 0
        Tindex7 = i-1;
        break;
    end
end

for i = 1:length(teltime7)
    if teltime7(i) > time7(Tindex7)
        telindex7 = i;
        break;
    end
end

teltime7 = teltime7 - time7(Tindex7);
time7 = time7 - time7(Tindex7);

Tindex8 = 1;
for i = 1:length(posz8)
    if posz8(i) > 0
        Tindex8 = i-1;
        break;
    end
end

for i = 1:length(teltime8)
    if teltime8(i) > time8(Tindex8)
        telindex8 = i;
        break;
    end
end

teltime8 = teltime8 - time8(Tindex8);
time8 = time8 - time8(Tindex8);

Tindex9 = 1;
for i = 1:length(posz9)
    if posz9(i) > 0
        Tindex9 = i-1;
        break;
    end
end

for i = 1:length(teltime9)
    if teltime9(i) > time9(Tindex9)
        telindex9 = i;
        break;
    end
end

teltime9 = teltime9 - time9(Tindex9);
time9 = time9 - time9(Tindex9);

Tindex10 = 1;
for i = 1:length(posz10)
    if posz10(i) > 0
        Tindex10 = i-1;
        break;
    end
end

for i = 1:length(teltime10)
    if teltime10(i) > time10(Tindex10)
        telindex10 = i;
        break;
    end
end

teltime10 = teltime10 - time10(Tindex10);
time10 = time10 - time10(Tindex10);

Tindex11 = 1;
for i = 1:length(posz11)
    if posz11(i) > 0
        Tindex11 = i-1;
        break;
    end
end

for i = 1:length(teltime11)
    if teltime11(i) > time11(Tindex11)
        telindex11 = i;
        break;
    end
end

teltime11 = teltime11 - time11(Tindex11);
time11 = time11 - time11(Tindex11);

Tindex12 = 1;
for i = 1:length(posz12)
    if posz12(i) > 0
        Tindex12 = i-1;
        break;
    end
end

for i = 1:length(teltime12)
    if teltime12(i) > time12(Tindex12)
        telindex12 = i;
        break;
    end
end

teltime12 = teltime12 - time12(Tindex12);
time12 = time12 - time12(Tindex12);

%% Plots

desPos = [0.1;0;0];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;

figure('Name','LQR Comparison')
subplot(2,1,1)
plot(lqrtime(lqrTindex:end),lqrposx(lqrTindex:end),'b')
hold on
plot(lqrtime(lqrTindex:end),lqrposz(lqrTindex:end),'b--')
plot(time1(Tindex1:end),posx1(Tindex1:end),'r')
plot(time1(Tindex1:end),posz1(Tindex1:end),'r--')
yline(1,'--k');
xlim([0 15])
ylim([0 0.15])
ylabel('Position(m)')
legend('lqr posx','lqr posz','linear mpc posx','linear mpc posz',...
    'location','best')
title('Position')

subplot(2,1,2)
plot(lqrtime(lqrTindex:end),(360/(2*pi))*lqrpitch(lqrTindex:end),'b')
hold on
plot(time1(Tindex1:end),(360/(2*pi))*pitch1(Tindex1:end),'r')
yline(0,'--k');
xlim([0 15])
% ylim([0 0.6])
ylabel('Pitch Angle (deg)')
legend('lqr pitch','linear pitch','location','best')
title('Pitch')

figure('Name','Motor Forces')
subplot(2,1,1)
plot(lqrteltime(lqrtelindex:end),mot1lqr(lqrtelindex:end))
hold on
plot(lqrteltime(lqrtelindex:end),mot2lqr(lqrtelindex:end))
plot(lqrteltime(lqrtelindex:end),mot3lqr(lqrtelindex:end))
plot(lqrteltime(lqrtelindex:end),mot4lqr(lqrtelindex:end))
xlim([0 8])
ylim([1.5 1.9])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('LQR Motor Forces')

subplot(2,1,2)
plot(teltime1(telindex1:end),mot11(telindex1:end))
hold on
plot(teltime1(telindex1:end),mot21(telindex1:end))
plot(teltime1(telindex1:end),mot31(telindex1:end))
plot(teltime1(telindex1:end),mot41(telindex1:end))
xlim([0 8])
ylim([1.5 1.9])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Unconstrained Linear MPC Motor Forces')

figure('Name','MPC Comparisons')
subplot(2,1,1)
plot(time1(Tindex1:end),posx1(Tindex1:end),'b')
hold on
plot(time1(Tindex1:end),posz1(Tindex1:end),'b--')
plot(time7(Tindex7:end),posx7(Tindex7:end),'r')
plot(time7(Tindex7:end),posz7(Tindex7:end),'r--')
yline(1,'--k');
xlim([0 15])
ylim([0 0.15])
ylabel('Position(m)')
legend('linear posx','linear posz','nonlinear posx','nonlinear posz',...
    'location','best')
title('Position')

subplot(2,1,2)
plot(time1(Tindex1:end),(360/(2*pi))*pitch1(Tindex1:end),'b')
hold on
plot(time7(Tindex7:end),(360/(2*pi))*pitch7(Tindex7:end),'r')
yline(0,'--k');
xlim([0 15])
% ylim([0 0.6])
ylabel('Pitch Angle (deg)')
legend('linear pitch','nonlinear pitch','location','best')
title('Pitch')

figure('Name','MPC Motor Forces')
subplot(2,1,1)
plot(teltime1(telindex1:end),mot11(telindex1:end))
hold on
plot(teltime1(telindex1:end),mot21(telindex1:end))
plot(teltime1(telindex1:end),mot31(telindex1:end))
plot(teltime1(telindex1:end),mot41(telindex1:end))
xlim([0 8])
ylim([1.5 1.9])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Unconstrained Linear MPC Motor Forces')

subplot(2,1,2)
plot(teltime7(telindex7:end),mot17(telindex7:end))
hold on
plot(teltime7(telindex7:end),mot27(telindex7:end))
plot(teltime7(telindex7:end),mot37(telindex7:end))
plot(teltime7(telindex7:end),mot47(telindex7:end))
xlim([0 8])
ylim([1.5 1.9])
ylabel('Force (N)')
legend('Motor 1', 'Motor 2','Motor 3', 'Motor 4')
title('Unconstrained Nonlinear MPC Motor Forces')
