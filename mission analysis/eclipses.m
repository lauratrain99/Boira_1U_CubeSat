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
filename = 'Data/eclipse_report.txt'; 
fileID = fopen(filename, 'r');

% Raw data
header = 3; 
data = readtable(filename, 'NumHeaderLines', header);
fclose(fileID);

% Eclipse duration
data = data(1:end-4,:);
duration = data(:,end);
duration = table2array(duration);

% Processing
[max_duration, index] = sort(duration);
min_duration = max_duration(1);
max_duration = max_duration(end); 
index = index(end);

fprintf('Minimum eclipse duration: %.2f min \n ', min_duration/60);
fprintf('Maximum eclipse duration: %.2f min \n', max_duration/60);
fprintf('Medium eclipse duration: %.2f min \n', mean(duration)/60);

%% Results
figure(1) 
plot(duration / 60)
xlabel('Eclipse event number')
ylabel('Eclipse duration (min)')
title('Eclipse duration throughout a year of mission')
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