% This script contains the parameters needed for running detumbling.slx
% Author: Laura Train

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

% Initial conditions
[r0, v0, k3, DCM_30] = coe2rv(3.986e+5, a, e, Omega, inc, omega, theta);
q30 = dcm2quat(DCM_30);
r0 = r0*1000;
v0 = v0*1000;

% initial angular velocity from launcher deploy
wx0 = 180; wy0 = -180; wz0 = 90;
w0 = deg2rad([wx0; wy0; wz0]) + n*k3;

% Initial attitude
euler0 = deg2rad([0,0,0]);
q0 = angle2quat(euler0(3)*pi/180,euler0(2)*pi/180,euler0(1)*pi/180,'ZYX');

% Inertia properties
Isc = [0.002, -3.759e-5, 1.047e-5;
       -3.759e-5, 0.002, -4.956e-6;
       1.047e-5, -4.956e-6, 0.002];

Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);

% Controller gain
K_Bdot = 10000;
 
% Maximum and minimum magnetic moment: magnetorquer
Max_magmom = 0.2; %Am^2
Min_magmom = -0.2; %Am^2

% Torque for magnetorquer;
Tmax = 10e-6*Max_magmom;


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

