%% HorizontalS Analysis
% The following data was obtained by calculating the optimal inputs for the
% control objective, and publishing all the inputs through a feedforward
% method.

%% Forward @ [4,0,1]

load('Forward_UMPC_posx.mat')
load('Forward_UMPC_posy.mat')
load('Forward_UMPC_posz.mat')
load('Forward_UMPC_time.mat')
load('Forward_UMPC_pitch.mat')
load('Forward_UMPC_roll.mat')
load('Forward_UMPC_velx.mat')
load('Forward_UMPC_vely.mat')
load('Forward_UMPC_velz.mat')
load('Forward_UMPC_yaw.mat')
load('Forward_UMPC_pitchrate.mat')
load('Forward_UMPC_radtime.mat')
load('Forward_UMPC_esttime.mat')
Uposx = posx;
Uposy = posy;
Uposz = posz;
Utime = time;
Upitch = pitch;
Uroll = roll;
Uyaw = yaw;
Uvelx = velx;
Uvely = vely;
Uvelz = velz;
Upitchrate = pitchrate;
Uradtime = radtime;
Uesttime = esttime;

load('Forward_MPC_posx.mat')
load('Forward_MPC_posy.mat')
load('Forward_MPC_posz.mat')
load('Forward_MPC_time.mat')
load('Forward_MPC_pitch.mat')
load('Forward_MPC_roll.mat')
load('Forward_MPC_velx.mat')
load('Forward_MPC_vely.mat')
load('Forward_MPC_velz.mat')
load('Forward_MPC_yaw.mat')
load('Forward_MPC_pitchrate.mat')
load('Forward_MPC_radtime.mat')
load('Forward_MPC_esttime.mat')

UTindex = 1;

for i = 1:length(Uposz)
    if Uposz(i) > 0
        UTindex = i-1;
        break;
    end
end

for i = 1:length(Uradtime)
    if Uradtime(i) > Utime(UTindex)
        URindex = i;
        break;
    end
end

Uradtime = Uradtime - Utime(UTindex);
Utime = Utime - Utime(UTindex);

for i = 1:length(Utime)
    if Utime(i) > 6
        UTindex1 = i;
        break;
    end
end

for i = 1:length(Uradtime)
    if Uradtime(i) > 6
        URindex1 = i - 1;
        break;
    end
end

Uradtime = Uradtime - Utime(UTindex1);
Utime = Utime - Utime(UTindex1);

desPos = [4;0;0];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = UTindex:length(Uposx)
    if Uposx(i) < lower(1) || Uposx(i) > upper(1)
        ss_index = i;
    end
end

settlingTime = Utime(ss_index);
forward_umpc_settlingTime = settlingTime;

[M,I] = max(Upitch);
disp('The max rate for unconstrained 2s horizon mpc is:')
disp((360/(2*pi))*M)

Tindex = 1;

for i = 1:length(posz)
    if posz(i) > 0
        Tindex = i-1;
        break;
    end
end

for i = 1:length(radtime)
    if radtime(i) > time(Tindex)
        Rindex = i;
        break;
    end
end

radtime = radtime - time(Tindex);
time = time - time(Tindex);

for i = 1:length(time)
    if time(i) > 6
        Tindex1 = i;
        break;
    end
end

for i = 1:length(radtime)
    if radtime(i) > 6
        Rindex1 = i - 1;
        break;
    end
end

radtime = radtime - time(Tindex1);
time = time - time(Tindex1);

desPos = [4;0;0];
lower = desPos - 0.02*desPos;
upper = desPos + 0.02*desPos;
ss_index = 1;

for i = Tindex:length(posx)
    if posx(i) < lower(1) || posx(i) > upper(1)
        ss_index1 = i;
    end
end

settlingTime = time(ss_index1);
forward_mpc_settlingTime = settlingTime;

[M,I] = max(pitch);
disp('The max rate for constrained is:')
disp((360/(2*pi))*M)

figure('Name','Position')
% subplot(2,1,1)
plot(Utime(UTindex1:end),Uposx(UTindex1:end),'b')
hold on
plot(time(Tindex1:end),posx(Tindex1:end),'b--')
% plot(Utime(UTindex1:end),Uposy(UTindex1:end),'g')
plot(Utime(UTindex1:end),Uposz(UTindex1:end),'r')
plot(time(Tindex1:end),posz(Tindex1:end),'r--')
xline(forward_umpc_settlingTime,'k');
xline(forward_mpc_settlingTime,'k--');
yline(lower(1),'--k');
yline(upper(1),'--k');
xlim([0 4])
ylim([0 4.5])
ylabel('Position(m)')
legend('UMPC posx','MPC posx','UMPC posz','UMPC posz','UMPC settling time',...
    'MPC settling time','location','best')
title('Reference at [4,0,1]')

% subplot(2,1,2)
% plot(time(Tindex1:end),posx(Tindex1:end),'b')
% hold on
% plot(time(Tindex1:end),posy(Tindex1:end),'g')
% plot(time(Tindex1:end),posz(Tindex1:end),'r')
% xline(forward_mpc_settlingTime,'k');
% yline(lower(1),'--k');
% yline(upper(1),'--k');
% xlim([0 4])
% ylim([0 4.5])
% xlabel('Time(s)')
% ylabel('Position(m)')
% legend('posx','posy','posz','settling time')
% title('Constrained MPC: Target [4,0,1]')

figure('Name','Orientation')
% subplot(2,1,1)
plot(Utime(UTindex1:end),(360/(2*pi))*Upitch(UTindex1:end),'r')
hold on
plot(time(Tindex1:end),(360/(2*pi))*pitch(Tindex1:end),'r--')
xlim([0 4])
ylim([-45 100])
yticks(-60:20:100)
xlabel('Time(s)')
ylabel('Angle (degree)')
legend('UMPC pitch', 'MPC pitch')
title('Reference at [4,0,1]')

% subplot(2,1,2)
% plot(time(Tindex1:end),(360/(2*pi))*pitch(Tindex1:end),'k')
% xlim([0 4])
% ylim([-45 100])
% yticks(-60:20:100)
% xlabel('Time(s)')
% ylabel('Angle (degree)')
% title('Constrained MPC: Target [4,0,1]')

figure('Name','Rates')
% subplot(2,1,1)
plot(Uradtime(URindex1:end),Upitchrate(URindex1:end),'b')
hold on
plot(radtime(Rindex1:end),pitchrate(Rindex1:end),'b--')
xlim([0 4])
ylim([-8 12])
xlabel('Time(s)')
ylabel('Rate (rad/s)')
legend('UMPC pitch', 'MPC pitch')
title('Reference at [4,0,1]')

% subplot(2,1,2)
% plot(radtime(Rindex1:end),pitchrate(Rindex1:end),'k')
% xlim([0 4])
% ylim([-8 12])
% xlabel('Time(s)')
% ylabel('rate (rad/s)')
% title('Constrained MPC: Target [4,0,1]')

ref1 = [0;0;1;0;0;0];
ref2 = [0;0;0];
ref3 = [4;0;1;0;0;0];
Q1 = 0.1*diag([20 20 16 58 58 3.9]);
Q2 = 0.1*diag([0.1 0.1 0.1]);
Ucost = 0;
Upose = [Uposx;Uposy;Uposz;Uroll;Upitch;Uyaw];
Uvel = [Uvelx;Uvely;Uvelz];
Ccost = 0;
pose = [posx;posy;posz;roll;pitch;yaw];
vel = [velx;vely;velz];

%% Unconstrained Cost
for i = UTindex:UTindex1-1
    Ucost = Ucost + (ref1-Upose(i))'*Q1*(ref1-Upose(i));
end

for i = UTindex1:ss_index
    Ucost = Ucost + (ref3-Upose(i))'*Q1*(ref3-Upose(i));
end

for i = URindex:URindex1-1
     Ucost = Ucost + (ref2-Uvel(i))'*Q2*(ref2-Uvel(i));
end

for i = URindex1:length(Uvelx)
     Ucost = Ucost + (ref2-Uvel(i))'*Q2*(ref2-Uvel(i));
     if Uesttime(i) > forward_umpc_settlingTime
         break;
     end
end



%% Constrained Cost
for i = Tindex:Tindex1-1
    Ccost = Ccost + (ref1-pose(i))'*Q1*(ref1-pose(i));
end

for i = Tindex1:ss_index
    Ccost = Ccost + (ref3-pose(i))'*Q1*(ref3-pose(i));
end

for i = Rindex:Rindex1-1
     Ccost = Ccost + (ref2-vel(i))'*Q2*(ref2-vel(i));
end

for i = Rindex1:length(velx)
     Ccost = Ccost + (ref2-vel(i))'*Q2*(ref2-vel(i));
     if esttime(i) > forward_mpc_settlingTime
         break;
     end
end

disp(Ucost)
disp(Ccost)

%% Unconstrained MPC 4s Forward @ [4,0,1]

% load('Forward_UMPC1_posx.mat')
% load('Forward_UMPC1_posy.mat')
% load('Forward_UMPC1_posz.mat')
% load('Forward_UMPC1_time.mat')
% load('Forward_UMPC1_pitch.mat')
% load('Forward_UMPC1_pitchrate.mat')
% load('Forward_UMPC1_teltime.mat')
% load('Forward_UMPC1_radtime.mat')
% load('Forward_UMPC1_mot1.mat')
% load('Forward_UMPC1_mot2.mat')
% load('Forward_UMPC1_mot3.mat')
% load('Forward_UMPC1_mot4.mat')
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
% for i = 1:length(radtime)
%     if radtime(i) > time(Tindex)
%         Rindex = i;
%         break;
%     end
% end
% 
% radtime = radtime - time(Tindex);
% time = time - time(Tindex);
% 
% for i = 1:length(time)
%     if time(i) > 6
%         Tindex = i;
%         break;
%     end
% end
% 
% for i = 1:length(radtime)
%     if radtime(i) > 6
%         Rindex = i - 1;
%         break;
%     end
% end
% 
% radtime = radtime - time(Tindex);
% time = time - time(Tindex);
% 
% desPos = [4;0;0];
% lower = desPos - 0.02*desPos;
% upper = desPos + 0.02*desPos;
% ss_index = 1;
% 
% for i = Tindex:length(posx)
%     if posx(i) < lower(1) || posx(i) > upper(1)
%         ss_index = i;
%     end
% end
% 
% settlingTime = time(ss_index);
% forward_umpc1_settlingTime = settlingTime;
% 
% telTindex = floor(Tindex/2);
% teltime = teltime - teltime(telTindex);
% 
% [M,I] = max(pitchrate);
% disp('The max rate for unconstrained 4s horizon mpc is:')
% disp(M)
% 
% figure('Name','Forward Unconstrained 4s MPC')
% subplot(3,1,1)
% plot(time(Tindex:end),posx(Tindex:end),'b')
% hold on
% plot(time(Tindex:end),posy(Tindex:end),'g')
% plot(time(Tindex:end),posz(Tindex:end),'r')
% xline(settlingTime,'k');
% yline(lower(1),'--k');
% yline(upper(1),'--k');
% xlim([0 4])
% ylim([0 4.5])
% xlabel('Time(s)')
% ylabel('Position(m)')
% legend('posx','posy','posz','settling time')
% title('Unconstrained MPC: Target [4,0,1]')
% 
% subplot(3,1,2)
% plot(time(Tindex:end),pitch(Tindex:end),'k')
% xlim([0 4])
% ylim([-0.8 2])
% xlabel('Time(s)')
% ylabel('rate (rad/s)')
% 
% subplot(3,1,3)
% plot(radtime(Rindex:end),pitchrate(Rindex:end),'k')
% xlim([0 4])
% ylim([-8 12])
% xlabel('Time(s)')
% ylabel('rate (rad/s)')

%% Rates Forward @ [4,0,1]

% load('Forward_Rates_posx.mat')
% load('Forward_Rates_posy.mat')
% load('Forward_Rates_posz.mat')
% load('Forward_Rates_time.mat')
% load('Forward_Rates_pitch.mat')
% load('Forward_Rates_pitchrate.mat')
% load('Forward_Rates_teltime.mat')
% load('Forward_Rates_radtime.mat')
% load('Forward_Rates_mot1.mat')
% load('Forward_Rates_mot2.mat')
% load('Forward_Rates_mot3.mat')
% load('Forward_Rates_mot4.mat')
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
% for i = 1:length(radtime)
%     if radtime(i) > time(Tindex)
%         Rindex = i;
%         break;
%     end
% end
% 
% radtime = radtime - time(Tindex);
% time = time - time(Tindex);
% 
% for i = 1:length(time)
%     if time(i) > 6
%         Tindex = i;
%         break;
%     end
% end
% 
% for i = 1:length(radtime)
%     if radtime(i) > 6
%         Rindex = i;
%         break;
%     end
% end
% 
% radtime = radtime - time(Tindex);
% time = time - time(Tindex);
% 
% desPos = [4;0;0];
% lower = desPos - 0.02*desPos;
% upper = desPos + 0.02*desPos;
% ss_index = 1;
% 
% for i = Tindex:length(posx)
%     if posx(i) < lower(1) || posx(i) > upper(1)
%         ss_index = i;
%     end
% end
% 
% settlingTime = time(ss_index);
% forward_rates_settlingTime = settlingTime;
% 
% telTindex = floor(Tindex/2);
% teltime = teltime - teltime(telTindex);
% 
% [M,I] = max(pitchrate);
% disp('The max rate for rates is:')
% disp(M)
% 
% figure('Name','Forward Rates')
% subplot(3,1,1)
% plot(time(Tindex:end),posx(Tindex:end),'b')
% hold on
% plot(time(Tindex:end),posy(Tindex:end),'g')
% plot(time(Tindex:end),posz(Tindex:end),'r')
% xline(settlingTime,'k');
% yline(lower(1),'--k');
% yline(upper(1),'--k');
% xlim([0 4])
% ylim([0 4.5])
% xlabel('Time(s)')
% ylabel('Position(m)')
% legend('posx','posy','posz','settling time')
% title('Rates: Target [4,0,1]')
% 
% subplot(3,1,2)
% plot(time(Tindex:end),pitch(Tindex:end),'k')
% xlim([0 4])
% ylim([-0.8 2])
% xlabel('Time(s)')
% ylabel('rate (rad/s)')
% 
% subplot(3,1,3)
% plot(radtime(Rindex:end),pitchrate(Rindex:end),'k')
% xlim([0 4])
% ylim([-8 12])
% xlabel('Time(s)')
% ylabel('rate (rad/s)')
