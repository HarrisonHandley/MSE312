%%  Linear Quadratic Gaussian Controller Design
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

D = [];

System = ss(A,B,C,D);


%%  Weighting Matrices
Q = [1  0   0
     0  10  0
     0  0   1000];
 
R = 0.1;

%%  Optimal Linear Gain Matrix
K = lqr(A,B,Q,R);

%%  Linear Controller Feedback
Aopt = (A-B*K);
Bopt = B;
Copt = C;
Dopt = D;

optsystem = ss(Aopt, Bopt, Copt, Dopt);

%%  Simulation
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


[tOL,yOL] = ode45(@(t,y)((A)*(y - [0; 0; pi])),t,u);
[topt,yopt] = ode45(@(topt,yopt)((A-B*K)*(yopt - [0; 0; pi])),t,u);

%%  Plotting
figure(1)
plot(tOL,yOL(:,3),'k','LineWidth',2)
hold on
plot(topt,yopt(:,3),'r', 'LineWidth', 2)
grid on
mylegend=legend ('Closed Loop', 'Optimized State Feedback');
set (mylegend,'FontSize',20,'Location','SouthEast')
myxlabel=xlabel ('time [s]');
myylabel=ylabel ('Angular Position [rad]');
set (myxlabel,'FontSize',24)
set (myylabel,'FontSize',24)
title('Closed Loop vs Optimized State Feedback', 'FontSize',24)


step(system, optsystem)