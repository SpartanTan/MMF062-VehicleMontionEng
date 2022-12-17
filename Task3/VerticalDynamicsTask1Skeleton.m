%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 1
%
% Add your own code where "%ADD YOUR CODE HERE" is stated
%
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

InitParameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.3
%
% Consider one single front wheel, identify sprung mass and unsprung mass

sprungMassFront =   %ADD YOUR CODE HERE
unsprungMassFront = %ADD YOUR CODE HERE

% Identify indiviual A and B matrix
Af =  %ADD YOUR CODE HERE
Bf =  %ADD YOUR CODE HERE

% Calculate transfer functions for 
% 1)front wheel Zr to Ride, 
% 2)front wheel Zr to Suspension travel and 
% 3) front wheel Zr to Tyre force

% matrices Zr to Ride,front wheel:
C1f = %ADD YOUR CODE HERE
D1f = %ADD YOUR CODE HERE

% matrices for Zr to Suspension travel, front wheel:
C2f = %ADD YOUR CODE HERE
D2f = %ADD YOUR CODE HERE

% matrices for Zr to Zr to Tyre force, front wheel:
C3f = %ADD YOUR CODE HERE
D3f = %ADD YOUR CODE HERE

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionFrontZrToRide(j,:) = %ADD YOUR CODE HERE
    transferFunctionFrontZrToTravel(j,:) = %ADD YOUR CODE HERE
    transferFunctionFrontZrToForce(j,:) = %ADD YOUR CODE HERE
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Consider one single rear wheel, identify sprung mass and unsprung mass

sprungMassRear = %ADD YOUR CODE HERE
unsprungMassRear = %ADD YOUR CODE HERE

% Identify indiviual A and B matrix
Ar =  %ADD YOUR CODE HERE
Br =  %ADD YOUR CODE HERE

% Calculate transfer functions for 
% 1) rear wheel Zr to Ride, 
% 2) rear wheel Zr to Suspension travel and 
% 3) rear wheel Zr to Tyre force

% matrices Zr to Ride, rear wheel:
C1r = %ADD YOUR CODE HERE
D1r = %ADD YOUR CODE HERE

% matrices for Zr to Suspension travel, rear wheel:
C2r = %ADD YOUR CODE HERE
D2r = %ADD YOUR CODE HERE

% matrices for Zr to Zr to Tyre force, rear wheel:
C3r = %ADD YOUR CODE HERE
D3r = %ADD YOUR CODE HERE

% Rear wheel
transferFunctionRearZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToForce = zeros(length(angularFrequencyVector),1);


for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionRearZrToRide(j,:) = %ADD YOUR CODE HERE
    transferFunctionRearZrToTravel(j,:) = %ADD YOUR CODE HERE
    transferFunctionRearZrToForce(j,:) = %ADD YOUR CODE HERE
end

% Plot the transfer functions
figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToRide)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToRide)),'--r');
axis([0 50 -10 60]);grid
legend('Front','Rear','Location','northwest');
xlabel('');%ADD YOUR CODE HERE
ylabel('');%ADD YOUR CODE HERE
title('Magnitude of transfer function Ride Comfort');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToTravel)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToTravel)),'--r');
axis([0 50 -50 10]);grid
legend('Front','Rear','Location','northwest');
xlabel('');%ADD YOUR CODE HERE
ylabel('');%ADD YOUR CODE HERE
title('Magnitude of transfer function Suspension Travel');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToForce)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToForce)),'--r');
axis([0 50 40 115]);grid
legend('Front','Rear','Location','northwest');
xlabel('');%ADD YOUR CODE HERE
ylabel('');%ADD YOUR CODE HERE
title('Magnitude of transfer function Road Grip');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.4
%
% Identify natural frequencies

% Front wheel
resonanceFreqFrontBounce = %ADD YOUR CODE HERE
resonanceFreqFrontHop = %ADD YOUR CODE HERE

% Rear wheel
resonanceFreqRearBounce = %ADD YOUR CODE HERE
resonanceFreqRearHop = %ADD YOUR CODE HERE
