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

% position and velocity -> assume Equatorial circular orbit
% r0 = [rmag; 0; 0];
% v0 = [1; sqrt(mu/rmag) + 1; 1];
year = 2024;
month = 1;
day = 1;
hour = 1;
min = 1;
sec = 1;
t0 = [year,month,day,hour,min,sec];

% assume a perturbation a 1% perturbation wrt Z axis angular velocity
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
Isc = [m/12 * (h^2 + d^2), 0, 0; 0, m/12 * (w^2 + d^2), 0; 0, 0, m/12 * (h^2 + w^2)];
Ix = Isc(1,1);
Iy = Isc(2,2);
Iz = Isc(3,3);

K_Bdot = 1000;
Max_magmom = 0.2;
Min_magmom = -0.2;