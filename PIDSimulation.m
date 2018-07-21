%%   Simulate P, PI, PD and PID
%%   Motor Parameters

Ra = 4.33;          %   [Ohms] Armature Resistance
La = 2.34e-3;        %   [Henry] Armature Inductance
K = 2.18e-2;         %   [Nm/A]  Motor Constant
J = 1.6e-6;      %   [kgm^2] Load Inertia
b = 0.02;           %   [Nms] Viscous Friction Constant


%%   Position Control Model
PosNum = [K];
PosDen = [(La*J) (Ra*J+b*La) (K*K+Ra*b)];
PosSys = tf(PosNum, PosDen)

%%   Speed Control Model
SpeedNum = [K];
SpeedDen = [(La*J) (Ra*J+b*La) (K*K+Ra*b) 0];
SpeedSys = tf(SpeedNum, SpeedDen)

%%   Speed Control of Motor Only
%   P

%   PI

%   PD

%   PID

%%   Speed Control of Motor, Truss and Magnet
%   P

%   PI

%   PD

%   PID

%%   Position Control of Motor Only
%   P

%   PI

%   PD

%   PID

%%   Position Control of Motor, Truss and Magnet
%   P

%   PI

%   PD

%   PID

