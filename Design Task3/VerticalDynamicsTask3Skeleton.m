%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 3

clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

InitParameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 3.1

numberOfTripsPerDay = totalDrivingTime/...
    (distanceSmoothRoad/vehicleVelocitySmooth+distanceRoughRoad/vehicleVelocityRough+distanceVeryRoughRoad/vehicleVelocityVeryRough);

% i)
% Calculate road spectrum
roadSpectrumSmooth = zeros(length(angularFrequencyVector),1);
roadSpectrumRough = zeros(length(angularFrequencyVector),1);
roadSpectrumVeryRough = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    roadSpectrumSmooth(j,:) = %ADD YOUR CODE HERE
    roadSpectrumRough(j,:) = %ADD YOUR CODE HERE
    roadSpectrumVeryRough(j,:) = %ADD YOUR CODE HERE
end

% Calculate transfer functions for front wheel Zr to Ride
% You can use result from Task 1

%ADD YOUR CODE HERE

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    transferFunctionFrontZrToRide(j,:) = %ADD YOUR CODE HERE 
end

% Calculate acceleration response spectrum for all roads

psdAccelerationSmooth = zeros(length(angularFrequencyVector),1);
psdAccelerationRough = zeros(length(angularFrequencyVector),1);
psdAccelerationVeryRough = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    psdAccelerationSmooth(j,:) = %ADD YOUR CODE HERE
    psdAccelerationRough(j,:) = %ADD YOUR CODE HERE
    psdAccelerationVeryRough(j,:) = %ADD YOUR CODE HERE
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ii)
% Calculate rms values weighted according to ISO2631 
[weightedRmsAccelerationSmooth] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationSmooth)
[weightedRmsAccelerationRough] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationRough)
[weightedRmsAccelerationVeryRough] = CalculateIsoWeightedRms(frequencyVector,psdAccelerationVeryRough)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% iii)
% Calculate time averaged vibration exposure value for 8h period
timeOnSmoothRoad = (distanceSmoothRoad / vehicleVelocitySmooth)*numberOfTripsPerDay
timeOnRoughRoad = (distanceRoughRoad / vehicleVelocityRough)*numberOfTripsPerDay
timeOnVeryRoughRoad = (distanceVeryRoughRoad / vehicleVelocityVeryRough)*numberOfTripsPerDay

timeWeightedMsAcceleration = %ADD YOUR CODE HERE

timeWeightedRmsAcceleration = sqrt(timeWeightedMsAcceleration);

disp(['timeWeightedRmsAcceleration =' num2str(timeWeightedRmsAcceleration),' m/s2 (1.15)',...
    ', numberOfTripsPerDay = ' num2str(numberOfTripsPerDay)]);
disp(['vehicleVelocitySmooth = ' num2str(vehicleVelocitySmooth*3.6),' km/h',...
    ', vehicleVelocityRough = ' num2str(vehicleVelocityRough*3.6),' km/h',...
    ', vehicleVelocityVeryRough = ' num2str(vehicleVelocityVeryRough*3.6),' km/h'])

