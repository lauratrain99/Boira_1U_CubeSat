clear;clc;close all;

addpath ../control
addpath ../conversion
addpath ../dynamics
addpath ../navigation

% orbital parameters
mu = 3.986e+14;
a = 6371 + 600;
e = 0;
Omega = 0;
inc = 0;
omega = 0;
theta = 0;

[r0, v0, k3, DCM_30] = coe2rv(3.986e+5, a, e, Omega, inc, omega, theta);
q30 = dcm2quat(DCM_30);
r0 = r0*1000;
v0 = v0*1000;

% orbital period
Torb = 2*pi*sqrt(norm(r0)^3/mu);

% angular velocity in-orbit
n = sqrt(mu/norm(r0)^3);

% Parameters used for secular variations of mag field
year = 2024;
month = 1;
day = 1;
hour = 0;
min = 0;
sec = 0;
t0 = [year,month,day,hour,min,sec];

% initial angular velocity
w0 = [0,0,n];

% initial euler angles
euler0 = [80,-80,80];
q0 = angle2quat(euler0(3)*pi/180,euler0(2)*pi/180,euler0(1)*pi/180,'ZYX');

% desired euler angles
eulerdes = [0,0,0];
qdes = angle2quat(eulerdes(3)*pi/180,eulerdes(2)*pi/180,eulerdes(1)*pi/180,'ZYX');
wdes = quatrotate(qdes,w0);

% Inertia properties
Isc = [0.002, -3.759e-5, 1.047e-5;
       -3.759e-5, 0.002, -4.956e-6;
       1.047e-5, -4.956e-6, 0.002];

Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);

% Maximum and minimum magnetic moment: magnetorquer
Max_magmom = 0.2; %Am^2
Min_magmom = -0.2; %Am^2

% Torque for magnetorquer;
Tmax = 10e-6*Max_magmom;

% desired angular velocities and times between burns
wx = sign(eulerdes(1)*pi/180 - euler0(1)*pi/180)*Tmax/Ix;
wy = sign(eulerdes(2)*pi/180 - euler0(2)*pi/180)*Tmax/Iy;
wz = sign(eulerdes(3)*pi/180 - euler0(3)*pi/180)*Tmax/Iz;

tol = 1e-10;

if abs(wx) < tol
    tx = 1;
else
    tx = abs( (euler0(1)*pi/180 - eulerdes(1)*pi/180) / wx);
end

if abs(wy) < tol
    ty = 1;
else
    ty = abs( (euler0(2)*pi/180 - eulerdes(2)*pi/180) / wy);
end

if abs(wz) < tol
    tz = 1;
else
    tz = abs( (euler0(3)*pi/180 - eulerdes(3)*pi/180) / wz);
end

% initial and final times for each current signal
t0x = 10;
tfx = t0x + tx;
t0y = 10 + tfx;
tfy = t0y + ty;
t0z = tfy + 10;
tfz = t0z + tz;

% roll transfer function
numX = [0, 0, 1];
denX = [Ix, 0]; 
sysX = tf(numX, denX);
% roll PID values
PID_paramsX = pidtune(sysX,'PID');
KpX = PID_paramsX.Kp;
KiX = PID_paramsX.Ki;
KdX = PID_paramsX.Kd;

numY = [0, 0, 1];
denY = [Iy, 0]; 
sysY = tf(numX, denX);
% pitch PID values
PID_paramsY = pidtune(sysX,'PID');
KpY = PID_paramsX.Kp;
KiY = PID_paramsX.Ki;
KdY = PID_paramsX.Kd;

numZ = [0, 0, 1];
denZ = [Iz, 0]; 
sysZ = tf(numX, denX);
% yaw PID values
PID_paramsZ = pidtune(sysX,'PID');
KpZ = PID_paramsX.Kp;
KiZ = PID_paramsX.Ki;
KdZ = PID_paramsX.Kd;

% set up Kp,Ki,Kd overall matrices
Kp = [KpX,0,0;0,KpY,0;0,0,KpZ];
Ki = [KiX,0,0;0,KiY,0;0,0,KiZ];
Kd = [KdX,0,0;0,KdY,0;0,0,KdZ];


%% SENSOR MODELING
misalign = 0.05; %deg
max_ref_temp = 70; %ºC

gyro.full_scale = 100; %deg/s
gyro.bias_ins = 8/3600; %deg/s
gyro.power_noise = ([0.12,0.17,0.17]/60).^2; %(deg/s)^2/Hz
gyro.quantization = 7.63e-8; %deg/s
gyro.temp_bias = 0.007; %deg/s/ºC

acc.full_scale = 5; %g
acc.bias_ins = 0.2/1000; %g
acc.power_noise = ([0.09,0.09,0.09]/(60*9.81)).^2; %g^2/Hz
acc.quantization = 0.25/1000; %g
acc.temp_bias = 0.05/1000;%g/ºC

mag.temp_bias = -0.3/100; % %/ºC
mag.quantization = 4.35e-3; % Gauss
mag.power_noise = ([2e-3,2e-3,2e-3]).^2; %Gauss^2/Hz


