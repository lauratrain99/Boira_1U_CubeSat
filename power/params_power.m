clear;clc;close all;

addpath ../control
addpath ../conversion
addpath ../dynamics
addpath ../navigation

% Initial conditions
% orbital parameter
mu = 3.986e+14;
a = 6371 + 600;
e = 0;
Omega = 0;
% Omega = 10*pi/180;
inc = 97.5*pi/180;
% inc = 0;
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

year = 2024;
month = 1;
day = 1;
hour = 0;
min = 0;
sec = 0;
t0 = [year,month,day,hour,min,sec];

% initial angular velocity from launcher deploy
% wx0 = 3; wy0 = 10; wz0 = -7;
% wx0 = 0; wy0 = 0; wz0 = 0;
% w0 = deg2rad([wx0; wy0; wz0]) + n*k3;
w0 = [0,0,n];


% euler0 = [20,-20,20];
euler0 = [0,0,0]
% eulerdes = [-45,0,0];
eulerdes = [0,0,0];

q0 = angle2quat(euler0(3)*pi/180,euler0(2)*pi/180,euler0(1)*pi/180,'ZYX');
qdes = angle2quat(eulerdes(3)*pi/180,eulerdes(2)*pi/180,eulerdes(1)*pi/180,'ZYX');
wdes = quatrotate(qdes,w0);

% Geometric and massic properties
% Iz>Iy, Iz>Ix stable configuration. h>w>d
% geometric dimensions of the S/C
m = 1.2;
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

% initial and final times for each thruster
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

% IMU ADIS16460
misalign = 0.05; %deg
max_ref_temp = 70; %ºC

gyro.full_scale = 100; %deg/s
gyro.bias_ins = 8/3600; %deg/s
gyro.power_noise = ([0.12,0.17,0.17]/60).^2; %(deg/s)^2/Hz
3*sqrt(gyro.power_noise*10)
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
3*sqrt(mag.power_noise*10)

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


