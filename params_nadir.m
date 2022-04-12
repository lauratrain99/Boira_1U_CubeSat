clear;clc;close all;

% Add paths

addpath dynamics/
addpath control/
addpath navigation/
addpath conversion/

% Initial conditions
% orbital parameter
mu = 3.986e+14;
a = 6371 + 600;
e = 0;
Omega = 0;
% inc = 50*pi/180;
inc = 0;
omega = 0;
theta = 0;

[r0, v0] = coe2rv(3.986e+5, a, e, Omega, inc, omega, theta);

r0 = r0*1000;
v0 = v0*1000;

% orbital period
Torb = 2*pi*sqrt(norm(r0)^3/mu);

year = 2024;
month = 1;
day = 1;
hour = 0;
min = 0;
sec = 0;
t0 = [year,month,day,hour,min,sec];

% initial angular velocity from launcher deploy
wx = 0; wy = 0; wz = 0;
w0 = deg2rad([wx; wy; wz]);
euler0 = [20,10,45];
% euler0 = deg2rad([0,0,0]);
q0 = angle2quat(euler0(3)*pi/180,euler0(2)*pi/180,euler0(1)*pi/180,'ZYX');

% Geometric and massic properties
% Iz>Iy, Iz>Ix stable configuration. h>w>d
% geometric dimensions of the S/C
m = 2;
w = 0.1;
h = 0.1;
d = 0.1;

% inertia tensor of the S/C
Ix = m/12 * (h^2 + d^2);
Iy = m/12 * (w^2 + d^2);
Iz = m/12 * (h^2 + w^2);
Ixy = 0;
Ixz = 0;
Iyz = 0;
Isc = [Ix, Ixy, Ixz; Ixy, Iy, Iyz; Ixz, Iyz, Iz];

% reaction wheels inertial tensor
Irw = [5.02e-5, 0, 0; 0, 9.41e-5, 0; 0, 0, 5.02e-5];

% Controller gain
K_Bdot = 1000;

% yaw transfer function
numZ = [0, 0, 1];
denZ = [Iz, 0, 0]; 
sysZ = tf(numZ, denZ);
% yaw PID values
PID_paramsZ = pidtune(sysZ,'PID');
KpZ = PID_paramsZ.Kp;
KiZ = PID_paramsZ.Ki;
KdZ = PID_paramsZ.Kd;

% pitch transfer function
numY = [0, 0, 1];
denY = [Iy, 0, 0]; 
sysY = tf(numY, denY);
% pitch PID values
PID_paramsY = pidtune(sysY,'PID');
KpY = PID_paramsY.Kp;
KiY = PID_paramsY.Ki;
KdY = PID_paramsY.Kd;

% roll transfer function
numX = [0, 0, 1];
denX = [Ix, 0, 0]; 
sysX = tf(numX, denX);
% roll PID values
PID_paramsX = pidtune(sysX,'PID');
KpX = PID_paramsX.Kp;
KiX = PID_paramsX.Ki;
KdX = PID_paramsX.Kd;

% set up Kp,Ki,Kd overall matrices
Kp = [KpX,0,0;0,KpY,0;0,0,KpZ];
Ki = [KiX,0,0;0,KiY,0;0,0,KiZ];
Kd = [KdX,0,0;0,KdY,0;0,0,KdZ];


% Thruster specifications
Fmax = 25e-3;
Larm = 5e-2;
t_thrust = 1;
Tmax = Fmax*Larm;

% desired angles
yaw = 0;
pitch = 0;
roll = 0;

% desired angular velocities and times between burns
wz = Tmax*t_thrust/Iz;
tz = abs(euler0(3) - deg2rad(yaw))/wz;
wy = Tmax*t_thrust/Iy;
ty = abs(euler0(2) - deg2rad(pitch))/wy;
wx = Tmax*t_thrust/Ix;
tx = abs(euler0(1) - deg2rad(roll))/wx;

% initial and final times for each thruster
t0z = 10;
tfz = t0z + tz;
t0y = tfz + 10;
tfy = t0y + ty;
t0x = tfy + 10;
tfx = t0x + tx;


% Maximum and minimum magnetic moment: magnetorquer
Max_magmom = 0.2; %Am^2
Min_magmom = -0.2; %Am^2

% IMU ADIS16460
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


