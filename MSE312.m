%%  MSE 312 State Space Model For Control System
close all, clc

%%  System Objectives
%   2% Settling Time (Ts)
%   exp^(-zeta*w_n*Ts) < 0.02
%   Ts = 4/(zeta*w_n) for underdamped
%   5deg Overshoot
%   P.O. > 100*exp^(-zeta*pi/sqrt(1-zeta^2)) underdamped system
%   P.O. = 5/180 x 100% = 2.8% 
%   zeta = 0.751223 [max]
%   cos-1(zeta) = 0.720883 rad
%   w_n = 2.662325302 [min]
%   TF = (w_n)^2/(s^2 + 2*zeta*w_n*s + w_n^2)

%   Simulate P, PI, PD and PID controllers using three different methods:
%   a. Simulink
%   b. MATLAB script using Control systems built-in functions (tf, feedback, etc.)
%   c. MATLAB script custom code (without using built-in functions)

%%  System Variable Definitions

Ra = 4.33;                        %   [Ohms] Armature Resistance
La = 2.34e-3;                     %   [Henry] Armature Inductance
K = 2.18e-2;                      %   [Nm/A]  Motor Constant
J = 1.6e-6 + 2.418269e-3 +1.1718445e-3 + 1.312933e-3;         %   [kgm^2] Load Inertia
%J = 1.6e-6;
b = 0.02;                         %   [Nms] Viscous Friction Constant

%%  State Variables
i = 0;          %   [A] Armature Current
theta = 0;      %   [rad] Motor Position
theta_dot = 0;  %   [rad/s] Motor Speed

%%  Input Variable 0 - 12V, 0 PWM being 0V, 100 PWM being 12V
V = 0;          %   [V] Voltage from H Bridge

%%  State Space Model
x = [theta
     theta_dot
     i];
 
A = [0      1       0
     0     -b/J     K/J
     0     -K/La     -Ra/La];
 
B = [0
     0
     1/La];
 
C = [1 0 0];
Cspeed = [0 1 0];
Call = [1 0 0  
        0 1 0];
Dall = [0
        0];
    
D = [];

PosSystem = ss(A,B,C,D);
SpeedSystem = ss(A,B,Cspeed,D);

%%  Transfer Function Initalization
s = tf('s');

%   Position Transfer Function
PosTF = tf(PosSystem);
PosTFNum = K/(La*J*s^3 + (Ra*J+b*La)*s^2 + (K*K+Ra*b)*s);
OLPosPoles = pole(PosTF);

%   Reduced Position Transfer Function
PosTFMin = minreal(PosTF*(s/max(abs(OLPosPoles)) + 1));

%   Speed Transfer Function
SpeedTF = tf(SpeedSystem);
SpeedTFNum = K/(La*J*s^2 + (Ra*J+b*La)*s + (K*K+Ra*b));
OLSpeedPoles = pole(SpeedTF);

%   Reduced Speed Transfer Function
SpeedTFMin = minreal(SpeedTF*(s/max(abs(OLSpeedPoles)) + 1));

% %%  Root Locus Plots
% %   Root locus of Speed Open Loop
% figure(1)
% subplot(2,2,1)
% rlocus(PosTF)
% title('Speed Control Open Loop Transfer Function')
% sgrid
% 
% %   Root Locus of Position Open Loop
% subplot(2,2,2)
% rlocus(SpeedTF)
% title('Position Control Open Loop Transfer Function')
% sgrid
% 
% %   Root Locus of Speed Minimized Open Loop
% subplot(2,2,3)
% rlocus(PosTFMin)
% title('Speed Control Open Loop Minimized Transfer Function')
% axis([ -10 10 -10 10])
% sgrid(0.751223, 0)
% sigrid(2)
% 
% %   Root Locus of Position Minimized Open Loop
% subplot(2,2,4)
% rlocus(SpeedTFMin)
% title('Position Control Open Loop Minimized Transfer Function')
% axis([ -10 10 -10 10])
% sgrid(0.751223, 0)
% sigrid(2)

%%  PID Simulations
%   Generate Input Signal
[u,t] = gensig('square', 4, 14, 0.00001);
lim = 14;
%   Processing on Input Signal
for i = 1:length(u)
    if i < length(u)/3
        u(i) = u(i)*0.75*pi;
    elseif i < length(u)*2/3
        u(i) = u(i)*pi/2;
    else
        u(i) = u(i)*pi/4;
    end     
end
u(i) = 0;
%%  P Controller

%   Position
%PKp = 45;   % No Truss
PKp = 6;    % With Truss
%   MATLAB Functions
PPosTF = tf(PKp, 1) * PosTF;
FBPPosTF = feedback(PPosTF, 1);
figure(2)
subplot(2,2,1)
[yPPM, tPPM, xPPM] = lsim(FBPPosTF,u,t);
plot(tPPM, yPPM)
hold on
plot(t, u)
PPMInfo = stepinfo(yPPM, tPPM, pi*0.75);
title(['Position - P Control MATLAB Functions | Settling Time:' num2str(PPMInfo.SettlingTime) ' Overshoot: ' num2str(PPMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PPosTFNum = PKp * PosTFNum;
FBPPosTFNum = PPosTFNum/(1 + PPosTFNum);
subplot(2,2,2)
[yPPN, tPPN, xPPN] = lsim(FBPPosTFNum,u,t);
plot(tPPN, yPPN)
hold on
plot(t, u)
PPNInfo = stepinfo(yPPN, tPPN, pi*0.75);
title(['Position - P Control Numeric | Settling Time:' num2str(PPNInfo.SettlingTime) ' Overshoot: ' num2str(PPNInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])

%   Speed
%SKp = 75;  % No Truss
SKp = 100;  % With Truss
%   MATLAB Functions
PSpeedTF = tf(SKp, 1) * SpeedTF;
FBPSpeedTF = feedback(PSpeedTF, 1);
subplot(2,2,3)
[ySPM, tSPM, xSPM] = lsim(FBPSpeedTF,u,t);
plot(tSPM, ySPM)
hold on
plot(t, u)
SPMInfo = stepinfo(ySPM, tSPM, pi*0.75);
title(['Speed - P Control MATLAB Functions | Settling Time:' num2str(SPMInfo.SettlingTime) ' Overshoot: ' num2str(SPMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PSpeedTFNum = SKp * SpeedTFNum;
FBPSpeedTFNum = PSpeedTFNum/(1 + PSpeedTFNum);
subplot(2,2,4)
[ySPN, tSPN, xSPN] = lsim(FBPSpeedTFNum,u,t);
plot(tSPN, ySPN)
hold on
plot(t, u)
SPNInfo = stepinfo(ySPN, tSPN, pi*0.75);
title(['Speed - P Control Numeric | Settling Time:' num2str(SPNInfo.SettlingTime) ' Overshoot: ' num2str(SPNInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])

%%  PI Controller

%   Position
% PKp = 80;   % No Truss
% PKi = 2;
PKp = 4.8;   % With Truss
PKi = 0.5;
%   MATLAB Functions
PIPosTF = tf([PKp PKi], [1 0]) * PosTF;
FBPIPosTF = feedback(PIPosTF, 1);
figure(3)
subplot(2,2,1)
[yPPIM, tPPIM, xPPIM] = lsim(FBPIPosTF,u,t);
plot(tPPIM, yPPIM)
hold on
plot(t, u)
PPIMInfo = stepinfo(yPPIM, tPPIM, pi*0.75);
title(['Position - PI Control MATLAB Functions | Settling Time:' num2str(PPIMInfo.SettlingTime) ' Overshoot: ' num2str(PPIMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PIPosTFNum = (PKp + PKi/s) * PosTFNum;
FBPIPosTFNum = PIPosTFNum/(1 + PIPosTFNum);
subplot(2,2,2)
[yPPIN, tPPIN, xPPIN] = lsim(FBPIPosTFNum,u,t);
plot(tPPIN, yPPIN)
hold on
plot(t, u)
PPINInfo = stepinfo(yPPIN, tPPIN, pi*0.75);
title(['Position - PI Control Numeric | Settling Time:' num2str(PPINInfo.SettlingTime) ' Overshoot: ' num2str(PPINInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])

%   Speed
% SKp = 50;   % No Truss
% SKi = 2;
SKp = 120;   % with Truss
SKi = 50;
%   MATLAB Functions
PISpeedTF = tf([SKp SKi], [1 0]) * SpeedTF;
FBPISpeedTF = feedback(PISpeedTF, 1);
subplot(2,2,3)
[ySPIM, tSPIM, xSPIM] = lsim(FBPISpeedTF,u,t);
plot(tSPIM, ySPIM)
hold on
plot(t, u)
SPIMInfo = stepinfo(ySPIM, tSPIM, pi*0.75);
title(['Speed - PI Control MATLAB Functions | Settling Time:' num2str(SPIMInfo.SettlingTime) ' Overshoot: ' num2str(SPIMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PISpeedTFNum = (SKp + SKi/s) * SpeedTFNum;
FBPISpeedTFNum = PISpeedTFNum/(1 + PISpeedTFNum);
subplot(2,2,4)
[ySPIN, tSPIN, xSPIN] = lsim(FBPISpeedTFNum,u,t);
plot(tSPIN, ySPIN)
hold on
plot(t, u)
SPINInfo = stepinfo(ySPIN, tSPIN, pi*0.75);
title(['Speed - PI Control Numeric | Settling Time:' num2str(SPINInfo.SettlingTime) ' Overshoot: ' num2str(SPINInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])

%%  PD Controller

%   Position
% PKp = 150;    % no truss
% PKd = 15;
PKp = 150;  % With Truss
PKd = 25;
%   MATLAB Functions
PDPosTF = tf([PKd PKp], [ 0 1 ]) * PosTF;
FBPDPosTF = feedback(PDPosTF, 1);
figure(4)
subplot(2,2,1)
[yPPDM, tPPDM, xPPDM] = lsim(FBPDPosTF,u,t);
plot(tPPDM, yPPDM)
hold on
plot(t, u)
PPDMInfo = stepinfo(yPPDM, tPPDM, pi*0.75);
title(['Position - PD Control MATLAB Functions | Settling Time:' num2str(PPDMInfo.SettlingTime) ' Overshoot: ' num2str(PPDMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PDPosTFNum = (PKd*s + PKp) * PosTFNum;
FBPDPosTFNum = PDPosTFNum/(1 + PDPosTFNum);
subplot(2,2,2)
[yPPDN, tPPDN, xPPDN] = lsim(FBPDPosTFNum,u,t);
plot(tPPDN, yPPDN)
hold on
plot(t, u)
PPDNInfo = stepinfo(yPPDN, tPPDN, pi*0.75);
title(['Position - PD Control Numeric | Settling Time:' num2str(PPDNInfo.SettlingTime) ' Overshoot: ' num2str(PPDNInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])

%   Speed
% SKp = 100;  % No Truss
% SKd = 25;
SKp = 100;  % With Truss
SKd = 50;
%   MATLAB Functions
PDSpeedTF = tf([SKd SKp], [0 1]) * SpeedTF;
FBPDSpeedTF = feedback(PDSpeedTF, 1);
subplot(2,2,3)
[ySPDM, tSPDM, xSPDM] = lsim(FBPDSpeedTF,u,t);
plot(tSPDM, ySPDM)
hold on
plot(t, u)
SPDMInfo = stepinfo(ySPDM, tSPDM, pi*0.75);
title(['Speed - PD Control MATLAB Functions | Settling Time:' num2str(SPDMInfo.SettlingTime) ' Overshoot: ' num2str(SPDMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PDSpeedTFNum = (SKd*s + SKp) * SpeedTFNum;
FBPDSpeedTFNum = PDSpeedTFNum/(1 + PDSpeedTFNum);
subplot(2,2,4)
[ySPDN, tSPDN, xSPDN] = lsim(FBPDSpeedTFNum,u,t);
plot(tSPDN, ySPDN)
hold on
plot(t, u)
SPDNInfo = stepinfo(ySPDN, tSPDN, pi*0.75);
title(['Speed - PD Control Numeric | Settling Time:' num2str(SPDNInfo.SettlingTime) ' Overshoot: ' num2str(SPDNInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])

%%  PID Controller

%   Position
% PKp = 150;  % No Truss
% PKi = 2;
% PKd = 10;
PKp = 50;  % With Truss
PKi = 2;
PKd = 10;
%   MATLAB Functions
PIDPosTF = tf([PKd PKp PKi], [1 0]) * PosTF;
FBPIDPosTF = feedback(PIDPosTF, 1);
figure(5)
subplot(2,2,1)
[yPPIDM, tPPIDM, xPPIDM] = lsim(FBPIDPosTF,u,t);
plot(tPPIDM, yPPIDM)
hold on
plot(t, u)
PPIDMInfo = stepinfo(yPPIDM, tPPIDM, pi*0.75);
title(['Position - PID Control MATLAB Functions | Settling Time:' num2str(PPIDMInfo.SettlingTime) ' Overshoot: ' num2str(PPIDMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PIDPosTFNum = (PKp + PKi/s + PKd*s) * PosTFNum;
FBPIDPosTFNum = PIDPosTFNum/(1 + PIDPosTFNum);
subplot(2,2,2)
[yPPIDN, tPPIDN, xPPIDN] = lsim(FBPIDPosTFNum,u,t);
plot(tPPIDN, yPPIDN)
hold on
plot(t, u)
PPIDNInfo = stepinfo(yPPIDN, tPPIDN, pi*0.75);
title(['Position - PID Control Numeric | Settling Time:' num2str(PPIDNInfo.SettlingTime) ' Overshoot: ' num2str(PPIDNInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])

%   Speed
% SKp = 250;  % No Truss
% SKi = 15;
% SKd = 15;
SKp = 250;  % With Truss
SKi = 15;
SKd = 15;
%   MATLAB Functions
PIDSpeedTF = tf([SKd SKp SKi], [1 0]) * SpeedTF;
FBPIDSpeedTF = feedback(PIDSpeedTF, 1);
subplot(2,2,3)
[ySPIDM, tSPIDM, xSPIDM] = lsim(FBPIDSpeedTF,u,t);
plot(tSPIDM, ySPIDM)
hold on
plot(t, u)
SPIDMInfo = stepinfo(ySPIDM, tSPIDM, pi*0.75);
title(['Speed - PID Control MATLAB Functions | Settling Time:' num2str(SPIDMInfo.SettlingTime) ' Overshoot: ' num2str(SPIDMInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])
%   Numeric
PIDSpeedTFNum = (SKp + SKi/s + SKd*s) * SpeedTFNum;
FBPIDSpeedTFNum = PIDSpeedTFNum/(1 + PIDSpeedTFNum);
subplot(2,2,4)
[ySPIDN, tSPIDN, xSPIDN] = lsim(FBPIDSpeedTFNum,u,t);
plot(tSPIDN, ySPIDN)
hold on
plot(t, u)
SPIDNInfo = stepinfo(ySPIDN, tSPIDN, pi*0.75);
title(['Speed - PID Control Numeric | Settling Time:' num2str(SPIDNInfo.SettlingTime) ' Overshoot: ' num2str(SPIDNInfo.Overshoot*1.35) ' [Degs]'])
xlim([0 lim])


%%  Comparison Plots
figure
plot(t,u,tSPIDM,ySPIDM,'-',tSPIDN,ySPIDN,'--',simout.time, simout.signals.values,':','LineWidth',2)
title('Speed - PID Control', 'Fontsize', 24)
legend('Reference','MATLAB Functions', 'MATLAB Script', 'Simulink')
xlabel('Time [s]', 'Fontsize', 24)
ylabel('Speed [rad/s]', 'Fontsize', 24)

%%  Final Plotting
SKp1 = 250;  % No Truss
SKi1 = 15;
SKd1 = 15;
%   MATLAB Functions
TF1 = tf([SKd1 SKp1 SKi1], [1 0]) * SpeedTF;
FBTF1 = feedback(TF1, 1);
[y1, t1, x1] = lsim(FBTF1,u,t);

SKp2 = 300;
SKi2 = 20;
SKd2 = 20;
%   MATLAB Functions
TF2 = tf([SKd2 SKp2 SKi2], [1 0]) * SpeedTF;
FBTF2 = feedback(TF2, 1);
[y2, t2, x2] = lsim(FBTF2,u,t);

SKp3 = 200;
SKi3 = 10;
SKd3 = 10;
%   MATLAB Functions
TF3 = tf([SKd3 SKp3 SKi3], [1 0]) * SpeedTF;
FBTF3 = feedback(TF3, 1);
[y3, t3, x3] = lsim(FBTF3,u,t);

figure
plot(t,u,t1, y1,'-', t2, y2,'--', t3, y3,':', 'Linewidth', 2)
title('Speed - PID Control | With Truss', 'Fontsize', 24)
legend('Reference','Nomainal','Increased Values', 'Decreased Values')
xlabel('Time [s]', 'Fontsize', 24)
ylabel('Speed [rad/s]', 'Fontsize', 24)
