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
inc = 50*pi/180;
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
hour = 1;
min = 1;
sec = 1;
t0 = [year,month,day,hour,min,sec];

% initial angular velocity from launcher deploy
wx = 5; wy = -3; wz = 4;
w0 = deg2rad([wx; wy; wz]);
q0 = angle2quat(0,0,0,'ZYX');

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

% Controller gain
K_Bdot = 1000;

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

