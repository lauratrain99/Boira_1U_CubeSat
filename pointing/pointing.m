% This script aims to simulate the measured sampling points gathered by the
% Boira radiometer of a 210 km diameter area during one year
% Author: Laura Train

clear;clc; close all;

title("Data gathered during one year")
set(groot, 'defaultTextInterpreter',            'latex');
set(groot, 'defaultAxesTickLabelInterpreter',   'latex'); 
set(groot, 'defaultLegendInterpreter',          'latex');
set(groot, 'defaultLegendLocation',             'northeast');


mag = 104.99*2;

RGT = 7;
repetitions = floor(365/RGT);
speed = 14.83;
times_oneside = floor(mag/speed) + 1;
a = linspace(0,2*pi,repetitions);



for i = 1:times_oneside
    xcircle(i,:) = mag/2*sin(a);
    ycircle(i,:) = mag/2*cos(a) + (i-1)*speed;
    r(i,:) = normrnd(0,mag/2/3 ,[1,repetitions]);
    v(i,:) = normrnd(0,mag/2/3 ,[1,repetitions]) + (i-1)*speed;
    
end

for i = (times_oneside + 1):(2*times_oneside)
    xcircle(i,:) = mag/2*sin(a);
    ycircle(i,:) = mag/2*cos(a) - ((i-times_oneside)-1)*speed;
    r(i,:) = normrnd(0,mag/2/3 ,[1,repetitions]);
    v(i,:) = normrnd(0,mag/2/3 ,[1,repetitions]) - ((i-times_oneside)-1)*speed;
    
end

for i = 1:(2*times_oneside)
    plot(r(i,:),v(i,:),'r*')
    hold on
    plot(xcircle(i,:),ycircle(i,:),'g')
    hold on
    axis equal
end

count = 1;
for i = 1:(2*times_oneside)
    for j = 1:repetitions
        if r(i,j) < max(xcircle(1,:)) && r(i,j) > min(xcircle(1,:)) && v(i,j) < max(ycircle(1,:)) && v(i,j) > min(ycircle(1,:))
            count = count + 1;
        end
    end
end

plot(xcircle(1,:),ycircle(1,:),'b','LineWidth',2)
xlabel("[km]")
ylabel("[km]")
title("Data gathered during one year")
axis equal
