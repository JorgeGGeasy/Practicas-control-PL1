clear, clc, close all
%% Configuracion
% Periodo de muestreo
Tm = 0.01; 

% Tiempo final de simulacion
Tend = 0.3; % seg
t = 0:Tm:Tend-Tm;
Lt = length(t);

% Senyal de referencia: escalon
ini_zeros = 5;
r = [zeros(1,ini_zeros) ones(1,Lt-ini_zeros)];

%% Sección: Modelo lineal
Tm = 0.01; % Periodo de muestreo
s = tf([1 0],1);

a = 1.38; % Polo dominante del motor
b = a*80; % 2 polo del motor (no dominante)
k = 2.9; % Ganancia del motor 
G = k*a/(s+a)*b/(s+b);
Gz = c2d(G,Tm);

%% Controlador PIDstd
Kp = 8;
Ti = 0.715;
Td = 0.004;
Cpidstd = pidstd(Kp,Ti,Td,'Ts',Tm,'IFormula','BackwardEuler','DFormula','BackwardEuler');

%% Controlador PID

Ki = Kp/Ti;
Kd = Kp*Td;
Cpid = pid(Kp,Ki,Kd,'Ts',Tm,'IFormula','BackwardEuler','DFormula','BackwardEuler');

%% Simulacion con modelos
sim('modelo_motor_P4_2.slx')