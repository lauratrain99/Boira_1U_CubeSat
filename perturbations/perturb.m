clear;clc;


Asp = 0.1*0.1;
phi = 1367;
c = 3e+8;
q = 0.8;
dev_sa = 0.01;
Tsp = phi/c * Asp * ( 1 + q) * cos(0) * dev_sa


rho = 8e-14;
CD = 2;
Aa = sqrt(0.1^2 + 0.1^2) ^2;
Vp = sqrt(3.986e+14/(500*1000 + 6371000));
dev_sa = 0.01;

Ta = 1/2 * rho * CD * Aa * Vp^2 * dev_sa


Tm = 7.69*10^15 / ((600 + 6371)*1000)^3 * 10^-3