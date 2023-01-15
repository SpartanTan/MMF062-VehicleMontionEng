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

sprungMassFront =  0.5*totalSprungMass*(wheelBase-distanceCogToFrontAxle)/wheelBase;
unsprungMassFront = 0.5*totalUnsprungMass*(wheelBase-distanceCogToFrontAxle)/wheelBase;

% Identify indiviual A and B matrix
Af =  [0,1,0,0;
    -frontWheelSuspStiff/sprungMassFront, -frontWheelSuspDamp/sprungMassFront, frontWheelSuspStiff/sprungMassFront,frontWheelSuspDamp/sprungMassFront;
    0,0,0,1;
    frontWheelSuspStiff/unsprungMassFront, frontWheelSuspDamp/unsprungMassFront, (-tireStiff-frontWheelSuspStiff)/unsprungMassFront, (-tireDamp-frontWheelSuspDamp)/unsprungMassFront];%ADD YOUR CODE HERE
Bf = [0;0;0;tireStiff]*1/unsprungMassFront;%ADD YOUR CODE HERE

%Bf = [0;0; 0; tireStiff/unsprungMassFront];
%% tye
% Calculate transfer functions for 
% 1)front wheel Zr to Ride, 
% 2)front wheel Zr to Suspension travel and 
% 3) front wheel Zr to Tyre force

% matrices Zr to Ride,front wheel:
% C1f = [-frontWheelSuspStiff/sprungMassFront -frontWheelSuspDamp/sprungMassFront frontWheelSuspStiff/sprungMassFront frontWheelSuspDamp/sprungMassFront]; % Zs
C1f = [1 0 0 0];
D1f = [0 0]; % 

% % matrices for Zr to Suspension travel, front wheel:
C2f = [-1 0 1 0];%ADD YOUR CODE HERE
D2f =  [0 0];%ADD YOUR CODE HERE
% 
% % matrices for Zr to Zr to Tyre force, front wheel:
C3f = [0 0 -1 0 ]*tireStiff;%ADD YOUR CODE HERE
D3f = [1 0]*tireStiff;%ADD YOUR CODE HERE

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    %temp =  C1f*inv(angularFrequencyVector(j)*eye(4)-Af)*Bf + D1f;
    transferFunctionFrontZrToRide(j,:) = -(angularFrequencyVector(j))^2*C1f*((1i*angularFrequencyVector(j)*eye(4)-Af)^(-1))*Bf + D1f(1,1);
    transferFunctionFrontZrToTravel(j,:) = C2f*((1i*angularFrequencyVector(j)*eye(4)-Af)^(-1))*Bf + D2f(1,1);
    transferFunctionFrontZrToForce(j,:) = C3f*((1i*angularFrequencyVector(j)*eye(4)-Af)^(-1))*Bf + D3f(1,1);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Consider one single rear wheel, identify sprung mass and unsprung mass

sprungMassRear = 0.5*totalSprungMass*(distanceCogToFrontAxle)/wheelBase;
unsprungMassRear = 0.5*totalUnsprungMass*(distanceCogToFrontAxle)/wheelBase;


% Identify indiviual A and B matrix
Ar =   [0,1,0,0;
    -rearWheelSuspStiff/sprungMassRear, -rearWheelSuspDamp/sprungMassRear, rearWheelSuspStiff/sprungMassRear,rearWheelSuspDamp/sprungMassRear;
    0,0,0,1;
    rearWheelSuspStiff/unsprungMassRear, rearWheelSuspDamp/unsprungMassRear, (-tireStiff-rearWheelSuspStiff)/unsprungMassRear, (-tireDamp-rearWheelSuspDamp)/unsprungMassRear];%ADD YOUR CODE HERE
% Br =   [0,0;
%         0,0;
%         0,0;
%         tireStiff,tireDamp]*1/unsprungMassFront;
Br = [0; tireDamp/unsprungMassRear; 0; tireStiff/unsprungMassRear];


% Calculate transfer functions for 
% 1) rear wheel Zr to Ride, 
% 2) rear wheel Zr to Suspension travel and 
% 3) rear wheel Zr to Tyre force

% matrices Zr to Ride, rear wheel:
% C1r = [-rearWheelSuspStiff/sprungMassRear -rearWheelSuspDamp/sprungMassRear rearWheelSuspStiff/sprungMassRear rearWheelSuspDamp/sprungMassRear]; %
C1r = [1 0 0 0];
D1r = [0 0]; % 

% % matrices for Zr to Suspension travel, rear wheel:
C2r = [-1 0 1 0];%ADD YOUR CODE HERE
D2r = [0 0];%ADD YOUR CODE HERE
% 
% % matrices for Zr to Zr to Tyre force, rear wheel:
C3r = [0 0 -1 0 ]*tireStiff; %ADD YOUR CODE HERE
D3r = [1 0]*tireStiff; %ADD YOUR CODE HERE

% Rear wheel
transferFunctionRearZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToTravel = zeros(length(angularFrequencyVector),1);
transferFunctionRearZrToForce = zeros(length(angularFrequencyVector),1);


for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    %temp = C1r*inv(angularFrequencyVector(j)*eye(4)-Ar)*Br + D1r;
    transferFunctionRearZrToRide(j,:) = -(angularFrequencyVector(j))^2*C1r*(1i*angularFrequencyVector(j)*eye(4)-Ar)^(-1)*Br + D1r(1,1);
    transferFunctionRearZrToTravel(j,:) = C2r*(1i*angularFrequencyVector(j)*eye(4)-Ar)^(-1)*Br + D2r(1,1);
    transferFunctionRearZrToForce(j,:) = C3r*(1i*angularFrequencyVector(j)*eye(4)-Ar)^(-1)*Br + D3r(1,1);
end

% Plot the transfer functions
figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToRide)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToRide)),'--r');
axis([0 50 -10 60]);grid
legend('Front','Rear','Location','northwest');
xlabel('Frequency [Hz]');%ADD YOUR CODE HERE
ylabel('|H(\omega)|Z_r \rightarrow Zsdotdot [Db]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Ride Comfort');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToTravel)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToTravel)),'--r');
axis([0 50 -50 10]);grid
legend('Front','Rear','Location','northwest');
xlabel('Frequency [Hz]');%ADD YOUR CODE HERE
ylabel('|H(\omega)|Z_r \rightarrow (Z_u-Z_s [Db]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Suspension Travel');

figure;
semilogx(frequencyVector,db(abs(transferFunctionFrontZrToForce)),'-b',...
    frequencyVector,db(abs(transferFunctionRearZrToForce)),'--r');
axis([0 50 40 115]);grid
legend('Front','Rear','Location','northwest');
xlabel('Frequency [Hz]');%ADD YOUR CODE HERE
ylabel('|H(\omega)|Z_r \rightarrow \Delta F_{rz} [Db]');%ADD YOUR CODE HERE
title('Magnitude of transfer function Road Grip');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.4
%
% Identify natural frequencies
clc;
%Front wheel
nominator = 1/(1/frontWheelSuspStiff+1/tireStiff);
resonanceFreqFrontBounce = sqrt(nominator/(sprungMassFront));%ADD YOUR CODE HERE
resonanceFreqFrontHop = sqrt((frontWheelSuspStiff+tireStiff)/(unsprungMassFront)); %ADD YOUR CODE HERE

% Rear wheel
nominator = 1/(1/rearWheelSuspStiff+1/tireStiff);
resonanceFreqRearBounce = sqrt(nominator/(sprungMassRear));%ADD YOUR CODE HERE
resonanceFreqRearHop = sqrt((rearWheelSuspStiff+tireStiff)/(unsprungMassRear));%ADD YOUR CODE HERE
clc;
toHz(resonanceFreqFrontBounce)
toHz(resonanceFreqFrontHop)
toHz(resonanceFreqRearBounce)
toHz(resonanceFreqRearHop)

function Hz = toHz(rad)
Hz = rad/(2*pi);
end