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
Cfull = eye(3);

D = [];

System = ss(A,B,C,D);
fullsys = ss(A,B,Cfull,D);

Controllability = rank(ctrb(A,B))
Observability = rank(obsv(A,C))

%%  Optimal Matrices For LQR
Q = [1  0   0
     0  0  0
     0  0   0];
 
R = 5;

%%  Optimal Linear Gain Matrix
K = lqr(System,Q,R);

%%  Kalman Filter 
%%  System Disturbances
Vd = 0.1*eye(3);    % Disturbance Covariance
Vn = 1;             % Noise Covariance

BF = [B Vd 0*B];    % New B Matrix For Kalman Filter (No measurement noise)

sysC = ss(A, BF, C, [0 0 0 0 Vn]);    % Includes new D matrix (no disturbance noise)

sysFullOutput = ss(A, BF, eye(3), zeros(3,size(BF,2))); % Outputs all state variables

%%  Build Kalman Filter
[L, P, E] = lqe(A, Vd, C, Vd, Vn);
sysKF = ss(A-L*C, [B L], eye(3), 0*[B L]);  % Kalman Filter Estimator

%%  Linear Controller Feedback
Aopt = (A-B*K);
Bopt = B;
Copt = C;
Dopt = D;

optsystem = ss(Aopt, Bopt, Copt, Dopt);

%%  Simulation Setup
[u,t] = gensig('square', 4, 14, 0.001);
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

uDIST = randn(3,size(t,1));
uNOISE = randn(size(t));

uAUG = [u'; Vd*Vd*uDIST; uNOISE'];
    
%%  Simulation
[y,t] = lsim(sysC, uAUG,t);
[xtrue, t] = lsim(sysFullOutput, uAUG, t);
[xopt,t] = lsim(optsystem, u, t);

[x,t] = lsim(sysKF, [u'; y'], t);

%%  Plotting
plot(t,xtrue,'-',t,x,'--','LineWidth',2)

figure
plot(t,y)
hold on
plot(t,xtrue(:,1),'r')
plot(t,x(:,1),'k--')

figure
plot(t,xopt,'-',t,u,'--')
% 
% 
% step(system, optsystem)