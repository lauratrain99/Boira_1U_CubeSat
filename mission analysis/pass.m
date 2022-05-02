%% BOIRA MISSION %% 
% Sergio, Mission Analysis MiSE
% 19/04/22

%% Eclipse Analysis
% This script provides an interface to study the eclipse periods throughout the mission. 

% All units are in S.I.

setup_path();
set_graphics();

%% File processing
% File data
filename = 'Data/pass_analysis_madrid.txt'; 
fileID = fopen(filename, 'r');

% Raw data
header = 3; 
data = readtable(filename, 'NumHeaderLines', header);
fclose(fileID);

% Pass duration
data = data(1:end-4,:);
duration = data(:,end);
duration = table2array(duration);

% Processing
MoS = 0.75;         % Margin of Safety

% Time between passes 
for i = 1:size(data,1)
    hour = char(data{i,4});
    month = select_month(char(data{i,2}));     
    date_vector = [data{i,3}, month, data{i,1}, str2double(hour(1:2)), str2double(hour(4:5)), str2double(hour(7:8))];
    dat = datetime(date_vector);
    time(i) = posixtime(dat);
end

time = sort(time);
await = time(2:end)-time(1:end-1);
await = await(await < 80*60 | await > 120*60 & await ~= 0);

%% Results
fprintf('Average pass duration: %.2f min \n', MoS*mean(duration)/60);
fprintf('Average delay: %.2f min \n', MoS*mean(await)/60);

figure(1) 
plot(duration / 60)
xlabel('Pass event number')
ylabel('Pass duration (min)')
title('Pass duration through the RGT cycle')
grid on; 

figure(2) 
plot(await / 60)
xlabel('Pass event number')
ylabel('Time between consecutive windows (min)')
title('Time between consecutive passes')
grid on; 

%% Auxiliary functions
% Set up path 
function setup_path()
    %Generate the search paths of the main subfolders of the program
    mainFolder = fileparts(which('eclipses'));

 	%Add paths
    addpath(genpath(mainFolder), '-end');
end

% Some cool graphics setup
function set_graphics()
    %Set graphical properties
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex'); 
    set(groot, 'defaultAxesFontSize', 11); 
    set(groot, 'defaultAxesGridAlpha', 0.3); 
    set(groot, 'defaultAxesLineWidth', 0.75);
    set(groot, 'defaultAxesXMinorTick', 'on');
    set(groot, 'defaultAxesYMinorTick', 'on');
    set(groot, 'defaultFigureRenderer', 'painters');
    set(groot, 'defaultLegendBox', 'off');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(groot, 'defaultLegendLocation', 'best');
    set(groot, 'defaultLineLineWidth', 1); 
    set(groot, 'defaultLineMarkerSize', 3);
    set(groot, 'defaultTextInterpreter','latex');
end

function [month] = select_month(month)
    switch (month)
        case 'Jan'
            month = 1;
        case 'Feb'
            month = 2;
        case 'Mar'
            month = 3;
        case 'Apr'
            month = 4;
        case 'May'
            month = 5;
        case 'Jun'
            month = 6;
        case 'Jul'
            month = 7;
        case 'Aug'
            month = 8;
        case 'Sep'
            month = 9;
        case 'Oct'
            month = 10;
        case 'Nov'
            month = 11;
        case 'Dec'
            month = 12;
    end
end