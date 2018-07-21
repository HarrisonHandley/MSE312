%%  MSE 312 State Space Model For Control System
clear all, close all, clc

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
J = 1.6e-6 + 2.418269e-3;         %   [kgm^2] Load Inertia
b = 0.02;                         %   [Nms] Viscous Friction Constant

%%  State Variables
i = 0;          %   [A] Armature Current
theta = 0;      %   [rad] Motor Position
theta_dot = 0;  %   [rad/s] Motor Speed

%%  Input Variable 0 - 12V, 0 PWM being 0V, 100 PWM being 12V
V = 0;          %   [V] Voltage from H Bridge

%%  State Space Model
x = [i
     theta
     theta_dot];
 
A = [-Ra/La  -K/La   0
     K/J     -b/J    0
     0        1      0];
 
B = [1/La
     0
     0];
 
C = [0 1 0];
Cspeed = [0 0 1];

D = [];

PosSystem = ss(A,B,C,D);
SpeedSystem = ss(A,B,Cspeed,D);

%%  Transfer Function Initalization
s = tf('s');

%   Position Transfer Function
PosTF = tf(PosSystem);
FBPosTF = feedback(PosSystem, 1);

PosTFNum = K/(La*J*s^2 + (Ra*J+b*La)*s + (K*K+Ra*b));
FBPosTFNum = PosTFNum/(1 + PosTFNum);
OLPosPoles = pole(PosTF);

%   Reduced Position Transfer Function
PosTFMin = minreal(PosTF*(s/max(abs(OLPosPoles)) + 1));

%   Speed Transfer Function
SpeedTF = tf(SpeedSystem);
FBSpeedTF = feedback(SpeedSystem, 1);
SpeedTFNum = K/(La*J*s^3 + (Ra*J+b*La)*s^2 + (K*K+Ra*b)*s);
FBSpeedTFNum = SpeedTFNum/(1 + SpeedTFNum);
OLSpeedPoles = pole(SpeedTF);

%   Reduced Speed Transfer Function
SpeedTFMin = minreal(SpeedTF*(s/max(abs(OLSpeedPoles)) + 1));

%%  Root Locus Plots
%   Root locus of Speed Open Loop
figure(1)
subplot(2,2,1)
rlocus(PosTF)
title('Speed Control Open Loop Transfer Function')
sgrid

%   Root Locus of Position Open Loop
subplot(2,2,2)
rlocus(SpeedTF)
title('Position Control Open Loop Transfer Function')
sgrid

%   Root Locus of Speed Minimized Open Loop
subplot(2,2,3)
rlocus(PosTFMin)
title('Speed Control Open Loop Minimized Transfer Function')
axis([ -10 10 -10 10])
sgrid(0.751223, 0)
sigrid(2)

%   Root Locus of Position Minimized Open Loop
subplot(2,2,4)
rlocus(SpeedTFMin)
title('Position Control Open Loop Minimized Transfer Function')
axis([ -10 10 -10 10])
sgrid(0.751223, 0)
sigrid(2)

%%  PID Simulations
%   Generate Input Signal
[u,t] = gensig('square', 4, 14, 0.01);
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
PKp = 5;
%   MATLAB Functions
FBPPosTF = tf(PKp, 1) * FBPosTF;
figure(2)
subplot(2,2,1)
[yPPM, tPPM, xPPM] = lsim(FBPPosTF,u,t);
plot(tPPM, yPPM)
hold on
plot(t, u)
PPMInfo = stepinfo(yPPM, tPPM, pi*0.75);
title(['Position - P Control MATLAB Functions | Settling Time:' num2str(PPMInfo.SettlingTime) ' Overshoot: ' num2str(PPMInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])
%   Numeric
FBPPosTFNum = PKp * FBPosTFNum;
subplot(2,2,2)
[yPPN, tPPN, xPPN] = lsim(FBPPosTFNum,u,t);
plot(tPPN, yPPN)
hold on
plot(t, u)
PPNInfo = stepinfo(yPPN, tPPN, pi*0.75);
title(['Position - P Control Numeric | Settling Time:' num2str(PPNInfo.SettlingTime) ' Overshoot: ' num2str(PPNInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])

%   Speed
SKp = 10;
%   MATLAB Functions
FBPSpeedTF = tf(SKp, 1) * FBSpeedTF;
subplot(2,2,3)
[ySPM, tSPM, xSPM] = lsim(FBPSpeedTF,u,t);
plot(tSPM, ySPM)
hold on
plot(t, u)
SPMInfo = stepinfo(ySPM, tSPM, pi*0.75);
title(['Speed - P Control MATLAB Functions | Settling Time:' num2str(SPMInfo.SettlingTime) ' Overshoot: ' num2str(SPMInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])
%   Numeric
FBPSpeedTFNum = SKp * FBSpeedTFNum;
subplot(2,2,4)
[ySPN, tSPN, xSPN] = lsim(FBPSpeedTFNum,u,t);
plot(tSPN, ySPN)
hold on
plot(t, u)
SPNInfo = stepinfo(ySPN, tSPN, pi*0.75);
title(['Speed - P Control Numeric | Settling Time:' num2str(SPNInfo.SettlingTime) ' Overshoot: ' num2str(SPNInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])

%%  PI Controller

%   Position
PKp = 1;
PKi = 1;
%   MATLAB Functions
FBPIPosTF = tf([PKp PKi], [1 0]) * FBPosTF;
figure(3)
subplot(2,2,1)
[yPPIM, tPPIM, xPPIM] = lsim(FBPIPosTF,u,t);
plot(tPPIM, yPPIM)
hold on
plot(t, u)
PPIMInfo = stepinfo(yPPIM, tPPIM, pi*0.75);
title(['Position - PI Control MATLAB Functions | Settling Time:' num2str(PPIMInfo.SettlingTime) ' Overshoot: ' num2str(PPIMInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])
%   Numeric
FBPIPosTFNum = (PKp + PKi/s) * FBPosTFNum;
subplot(2,2,2)
[yPPIN, tPPIN, xPPIN] = lsim(FBPIPosTFNum,u,t);
plot(tPPIN, yPPIN)
hold on
plot(t, u)
PPINInfo = stepinfo(yPPIN, tPPIN, pi*0.75);
title(['Position - PI Control Numeric | Settling Time:' num2str(PPINInfo.SettlingTime) ' Overshoot: ' num2str(PPINInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])

%   Speed
SKp = 1;
SKi = 1;
%   MATLAB Functions
FBPISpeedTF = tf([SKp SKi], [1 0]) * FBSpeedTF;
subplot(2,2,3)
[ySPIM, tSPIM, xSPIM] = lsim(FBPISpeedTF,u,t);
plot(tSPIM, ySPIM)
hold on
plot(t, u)
SPIMInfo = stepinfo(ySPIM, tSPIM, pi*0.75);
title(['Speed - PI Control MATLAB Functions | Settling Time:' num2str(SPIMInfo.SettlingTime) ' Overshoot: ' num2str(SPIMInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])
%   Numeric
FBPISpeedTFNum = (SKp + SKi/s) * FBSpeedTFNum;
subplot(2,2,4)
[ySPIN, tSPIN, xSPIN] = lsim(FBPISpeedTFNum,u,t);
plot(tSPIN, ySPIN)
hold on
plot(t, u)
SPINInfo = stepinfo(ySPIN, tSPIN, pi*0.75);
title(['Speed - PI Control Numeric | Settling Time:' num2str(SPINInfo.SettlingTime) ' Overshoot: ' num2str(SPINInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])

%%  PID Controller

%   Position
PKp = 1;
PKi = 1;
PKd = 1;
%   MATLAB Functions
FBPIDPosTF = tf([PKd PKp PKi], [1 0]) * FBPosTF;
figure(4)
subplot(2,2,1)
[yPPIDM, tPPIDM, xPPIDM] = lsim(FBPIDPosTF,u,t);
plot(tPPIDM, yPPIDM)
hold on
plot(t, u)
PPIDMInfo = stepinfo(yPPIDM, tPPIDM, pi*0.75);
title(['Position - PID Control MATLAB Functions | Settling Time:' num2str(PPIDMInfo.SettlingTime) ' Overshoot: ' num2str(PPIDMInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])
%   Numeric
FBPIDPosTFNum = (PKp + PKi/s + PKd*s) * FBPosTFNum;
subplot(2,2,2)
[yPPIDN, tPPIDN, xPPIDN] = lsim(FBPIDPosTFNum,u,t);
plot(tPPIDN, yPPIDN)
hold on
plot(t, u)
PPIDNInfo = stepinfo(yPPIDN, tPPIDN, pi*0.75);
title(['Position - PID Control Numeric | Settling Time:' num2str(PPIDNInfo.SettlingTime) ' Overshoot: ' num2str(PPIDNInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])

%   Speed
SKp = 1;
SKi = 100;
SKd = 1;
%   MATLAB Functions
FBPIDSpeedTF = tf([SKd SKp SKi], [1 0]) * FBSpeedTF;
subplot(2,2,3)
[ySPIDM, tSPIDM, xSPIDM] = lsim(FBPIDSpeedTF,u,t);
plot(tSPIDM, ySPIDM)
hold on
plot(t, u)
SPIDMInfo = stepinfo(ySPIDM, tSPIDM, pi*0.75);
title(['Speed - PID Control MATLAB Functions | Settling Time:' num2str(SPIDMInfo.SettlingTime) ' Overshoot: ' num2str(SPIDMInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])
%   Numeric
FBPIDSpeedTFNum = (SKp + SKi/s + SKd*s) * FBSpeedTFNum;
subplot(2,2,4)
[ySPIDN, tSPIDN, xSPIDN] = lsim(FBPIDSpeedTFNum,u,t);
plot(tSPIDN, ySPIDN)
hold on
plot(t, u)
SPIDNInfo = stepinfo(ySPIDN, tSPIDN, pi*0.75);
title(['Speed - PID Control Numeric | Settling Time:' num2str(SPIDNInfo.SettlingTime) ' Overshoot: ' num2str(SPIDNInfo.Overshoot*180/pi) ' [Degs]'])
xlim([0 lim])

%%  Speed Control Model Analysis
%%  Desired Pole and Zero Placement
%   P

%%  Optimal Matrices
Q = [1  0   0
     0  10  0
     0  0   1000];
 
R = 0.1;

% %%  Optimal Linear Gain Matrix
% K = lqr(A,B,Q,R);
% 
% %%  Linear Controller Feedback
% Aopt = (A-B*K);
% Bopt = B;
% Copt = C;
% Dopt = D;
% 
% optsystem = ss(Aopt, Bopt, Copt, Dopt);
% 
% tspan = 0:0.001:10;
% y0 = [0
%       0
%       0];
% 
% [t,y] = ode45(@(t,y)((A)*(y - [0; 0; pi])),tspan,y0);
% [topt,yopt] = ode45(@(topt,yopt)((A-B*K)*(yopt - [0; 0; pi])),tspan,y0);
% 
% figure(1)
% plot(t,y(:,3),'k','LineWidth',2)
% hold on
% plot(topt,yopt(:,3),'r', 'LineWidth', 2)
% grid on
% mylegend=legend ('Closed Loop', 'Optimized State Feedback');
% set (mylegend,'FontSize',20,'Location','SouthEast')
% myxlabel=xlabel ('time [s]');
% myylabel=ylabel ('Angular Position [rad]');
% set (myxlabel,'FontSize',24)
% set (myylabel,'FontSize',24)
% title('Closed Loop vs Optimized State Feedback', 'FontSize',24)


%step(system, optsystem)