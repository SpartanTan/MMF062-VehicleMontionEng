%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, initialize parameters
% 
% Add your own code where "%ADD YOUR CODE HERE" is stated
%
clear all;
close all;
clc;

set(0,'DefaultAxesFontSize',12)
set(0,'DefaultAxesLineWidth',2)
set(0,'defaultlinelinewidth',2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle parameters [SI units]
%
totalMass =               1860; %[kg]
totalUnsprungMass =       185; %[kg]
totalSprungMass =         totalMass-totalUnsprungMass;
wheelBase =               2.675;
distanceCogToFrontAxle =  0.4*wheelBase;
frontWheelSuspStiff =     30800;
rearWheelSuspStiff =      29900;
frontWheelSuspDamp =      4500;
rearWheelSuspDamp =       3500;
tireStiff =               230000;
tireDamp =                0;

% Other parameters Task 1
%
frequencyVector = [0.1 : 0.01 : 50]';
angularFrequencyVector = 2 * pi * frequencyVector;
deltaAngularFrequency = 2 * pi * 0.01;

% Uncomment following from Task 2 and 3

% Other parameters Task 2
%
roadDisplacementAmplitude = 1;
roadSeverity =            10*10^-6;
vehicleVelocity =         80/3.6;
roadWaviness =            2.5;
frontWheelSuspDampVector = [1000 : 100 : 9000];
frontWheelSuspStiffVector = [0.5 0.75 1 1.25 1.5 2]*frontWheelSuspStiff;

% Other parameters Task 3
%
roadSeveritySmooth =      1e-6;
roadSeverityRough =       10e-6;
roadSeverityVeryRough =   100e-6;
roadWavinessSmooth =      3;
roadWavinessRough =       2.5;
roadWavinessVeryRough =   2;

totalDrivingTime = 8*60*60;               % 8h driving period
totalDistance = 160E3;                    % total distance of route (there and back)
distanceSmoothRoad = 0.70*totalDistance;   % distribution of different road types
distanceRoughRoad = 0.175*totalDistance;
distanceVeryRoughRoad = 0.125*totalDistance;

vehicleVelocitySmooth = 110/3.6;    % max allowed vehicle velocity on any road type 100km/h
vehicleVelocityRough = 99/3.6;
vehicleVelocityVeryRough = 14/3.6;

