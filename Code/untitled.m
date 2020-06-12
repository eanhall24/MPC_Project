clear
clc

desAcc = 0;
desPos = 0.5;
estPos = 0;
natFreq = 2;
dampingRatio = 0.7;
desVel = 0;
estVel = 0;
g = 9.81;
timeconst = 0.0914;

accel = (desPos - estPos) * natFreq * natFreq...
        + (desVel - estVel) * 2 * natFreq * dampingRatio + desAcc;
Cpitch = accel/g;
Cpitchrate = -1/timeconst*(0 - Cpitch);

disp(Cpitchrate)