% This script contains the parameters needed for running power.slx
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
inc = 97.5*pi/180;
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

% initial angular velocity from launcher deploy
w0 = [0,0,n];

% attitude initial conditions
euler0 = [0,0,0];
q0 = angle2quat(euler0(3)*pi/180,euler0(2)*pi/180,euler0(1)*pi/180,'ZYX');


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


%%


set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');

figure()
plot(out.power_dawndusk.Time, out.power_dawndusk.Data,'r')
% , ...)
%     out.power_noonmid.Time(1:1845), 6.3*ones(1845,1),'b', ...
%     out.power_noonmid.Time(4000:end), 6.3*ones(1956,1),'b')
title("Power from solar arrays one orbit. Noon-midnight")
text(2500, 3,  "ECLIPSE")
% t = text(500, 6.5,  "Needed")
% p = text(4500, 6.5,  "Needed")
% t.Color = "blue"
% p.Color = "blue"
xlabel("Time [s]")
ylabel("Power [W]")
xlim([-200, 6000])
grid minor

powerdawndusk(:,1) = out.power_dawndusk.Time;
powerdawndusk(:,2) = out.power_dawndusk.Data;
save('powerdawndusk.mat','powerdawndusk')

% powernoonmid(:,1) = out.power_noonmid.Time;
% powernoonmid(:,2) = out.power_noonmid.Data;
% save('powernoonmid.mat','powernoonmid')
%%

faceX = out.faceX;
save('faceX.mat','faceX')

faceY = out.faceY;
save('faceY.mat','faceY')

faceZ = out.faceZ;
save('faceZ.mat','faceZ')

facepayload = out.facepayload;
save('facepayload.mat','facepayload')

faceminusY = out.faceminusY;
save('faceminusY.mat','faceminusY')

faceminusZ = out.faceminusZ;
save('faceminusZ.mat','faceminusZ')


