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

InitParametersSkeleton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.3
%
% Consider one single front wheel, identify sprung mass and unsprung mass

sprungMassFront = 0.5*totalSprungMass*(wheelBase-distanceCogToFrontAxle)/wheelBase;
unsprungMassFront = 0.5*totalUnsprungMass*(wheelBase-distanceCogToFrontAxle)/wheelBase;

% Identify indiviual A and B matrix
Af = [0,1,0,0;
    -frontWheelSuspStiff/sprungMassFront, -frontWheelSuspDamp/sprungMassFront, frontWheelSuspStiff/sprungMassFront,frontWheelSuspDamp/sprungMassFront;
    0,0,0,1;
    frontWheelSuspStiff/unsprungMassFront, frontWheelSuspDamp/unsprungMassFront, (-tireStiff-frontWheelSuspStiff)/unsprungMassFront, (-tireDamp-frontWheelSuspDamp)/unsprungMassFront];
Bf = [0;0;0;tireStiff/unsprungMassFront];

% Calculate transfer functions for 
% 1)front wheel Zr to Ride, 
% 2)front wheel Zr to Suspension travel and 
% 3) front wheel Zr to Tyre force

% matrices Zr to Ride,front wheel:
%C1f = [-frontWheelSuspStiff/sprungMassFront, -frontWheelSuspDamp/sprungMassFront, frontWheelSuspStiff/sprungMassFront,frontWheelSuspDamp/sprungMassFront];
C1f= [1 0 0 0];
D1f = 0;

% matrices for Zr to Suspension travel, front wheel:
C2f = [-1 0 1 0];
D2f = 0;

% matrices for Zr to Zr to Tyre force, front wheel:
C3f = [0 0 -tireStiff 0];
D3f = tireStiff;

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionFrontZrToRide(j,:) = -angularFrequencyVector(j)^2*C1f*inv((1i*angularFrequencyVector(j)*eye(4)-Af))*Bf+D1f;
    transferFunctionFrontZrToTravel(j,:) = C2f*inv((1i*angularFrequencyVector(j)*eye(4)-Af))*Bf+D2f;
    transferFunctionFrontZrToForce(j,:) = C3f*inv((1i*angularFrequencyVector(j)*eye(4)-Af))*Bf+D3f;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Consider one single rear wheel, identify sprung mass and unsprung mass

sprungMassRear = 0.5*totalSprungMass*distanceCogToFrontAxle/wheelBase;
unsprungMassRear = 0.5*totalUnsprungMass*distanceCogToFrontAxle/wheelBase;

% Identify indiviual A and B matrix
Ar = [0,1,0,0;
    -rearWheelSuspStiff/sprungMassRear, -rearWheelSuspDamp/sprungMassRear, rearWheelSuspStiff/sprungMassRear,rearWheelSuspDamp/sprungMassRear;
    0,0,0,1;
    rearWheelSuspStiff/unsprungMassRear, rearWheelSuspDamp/unsprungMassRear, (-tireStiff-rearWheelSuspStiff)/unsprungMassRear, (-tireDamp-rearWheelSuspDamp)/unsprungMassRear];
Br = [0;0;0;tireStiff/unsprungMassRear];
% Calculate transfer functions for 
% 1) rear wheel Zr to Ride, 
% 2) rear wheel Zr to Suspension travel and 
% 3) rear wheel Zr to Tyre force

% matrices Zr to Ride, rear wheel:
C1r= [1 0 0 0];
D1r = 0;

% matrices for Zr to Suspension travel, rear wheel:
C2r = [-1 0 1 0];
D2r = 0;

% matrices for Zr to Zr to Tyre force, rear wheel:
C3r = [0 0 -tireStiff 0];
D3r = tireStiff;

% Rear wheel
transferFunctionRearZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToForce = zeros(length(angularFrequencyVector),1);


for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionRearZrToRide(j,:) = -angularFrequencyVector(j)^2*C1r*inv((1i*angularFrequencyVector(j)*eye(4)-Ar))*Br+D1r;
    transferFunctionRearZrToTravel(j,:) = C2r*inv((1i*angularFrequencyVector(j)*eye(4)-Ar))*Br+D2r;
    transferFunctionRearZrToForce(j,:) = C3r*inv((1i*angularFrequencyVector(j)*eye(4)-Ar))*Br+D3r;
end

% Plot the transfer functions
figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToRide)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToRide)),'--r');
axis([0 50 -10 60]);grid
legend('Front','Rear','Location','northwest');
xlabel('vx [m/s]');%ADD YOUR CODE HERE
ylabel('amplification [m/s^2/m]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Ride Comfort');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToTravel)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToTravel)),'--r');
axis([0 50 -50 10]);grid
legend('Front','Rear','Location','northwest');
xlabel('vx [m/s]');%ADD YOUR CODE HERE
ylabel('amplification [m/m]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Suspension Travel');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToForce)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToForce)),'--r');
axis([0 50 40 115]);grid
legend('Front','Rear','Location','northwest');
xlabel('vx [m/s]');%ADD YOUR CODE HERE
ylabel('amplification [N/m]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Road Grip');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.4
%
% Identify natural frequencies

% Front wheel
resonanceFreqFrontBounce = sqrt((1/(1/frontWheelSuspStiff+1/tireStiff))/sprungMassFront);
resonanceFreqFrontHop = sqrt((frontWheelSuspStiff+tireStiff)/unsprungMassFront);

% Rear wheel
resonanceFreqRearBounce = sqrt((1/(1/rearWheelSuspStiff+1/tireStiff))/sprungMassRear);
resonanceFreqRearHop = sqrt((rearWheelSuspStiff+tireStiff)/unsprungMassRear);

Hz_FrontBounce=toHz(resonanceFreqFrontBounce)
Hz_FrontHop=toHz(resonanceFreqFrontHop)
Hz_rearBounce=toHz(resonanceFreqRearBounce)
Hz_rearHop=toHz(resonanceFreqRearHop)
function Hz=toHz(rad)
    Hz=rad/(2*pi);
end
