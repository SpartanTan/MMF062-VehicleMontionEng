%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2020
% Vertical assignment, Task 2
% 
%
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load parameters from file "InitParameters.m"

InitParametersSkeleton
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 2.1
%
% Front wheel
%
% Calculate road spectrum
roadSpectrum = zeros(length(angularFrequencyVector),1);

for i = 1 : length(angularFrequencyVector)
    roadSpectrum(i,:) = (vehicleVelocity^(roadWaviness-1))*roadSeverity*(angularFrequencyVector(i)^(-roadWaviness));
end

% Calculate transfer functions for front wheel Zr to Ride and Tyre force
% You can use the code from Task 1

% Consider one single front wheel
sprungMassFront = 0.5*totalSprungMass*(wheelBase-distanceCogToFrontAxle)/wheelBase;
unsprungMassFront = 0.5*totalUnsprungMass*(wheelBase-distanceCogToFrontAxle)/wheelBase;

% Identify A and B matrix
Af = [0,1,0,0;
    -frontWheelSuspStiff/sprungMassFront, -frontWheelSuspDamp/sprungMassFront, frontWheelSuspStiff/sprungMassFront,frontWheelSuspDamp/sprungMassFront;
    0,0,0,1;
    frontWheelSuspStiff/unsprungMassFront, frontWheelSuspDamp/unsprungMassFront, (-tireStiff-frontWheelSuspStiff)/unsprungMassFront, (-tireDamp-frontWheelSuspDamp)/unsprungMassFront];
Bf = [0;0;0;tireStiff/unsprungMassFront];

% Calculate transfer functions for 
% 1)front wheel Zr to Ride, 
% 3) front wheel Zr to Tyre force
% similar to Task 1 define C and D matrices for both cases
C1f= [1 0 0 0];
D1f = 0;

C3f = [0 0 -tireStiff 0];
D3f = tireStiff;

transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);

for j = 1 : length(angularFrequencyVector)
    % Calculate H(w) not the absolut value |H(w)|
    transferFunctionFrontZrToRide(j,:) = -angularFrequencyVector(j)^2*C1f*inv((1i*angularFrequencyVector(j)*eye(4)-Af))*Bf+D1f;
    transferFunctionFrontZrToForce(j,:) = C3f*inv((1i*angularFrequencyVector(j)*eye(4)-Af))*Bf+D3f;
end

% Calculate acceleration and tyre force response spectrum
psdAcceleration = zeros(length(angularFrequencyVector),1);
psdForce = zeros(length(angularFrequencyVector),1);

for m = 1 : length(angularFrequencyVector)
    psdAcceleration(m,:) = abs(transferFunctionFrontZrToRide(m))^2*roadSpectrum(m);
    psdForce(m,:) = abs(transferFunctionFrontZrToForce(m))^2*roadSpectrum(m);
end

% Calculate rms values of acceleration and tyre force
msAcceleration = 0;
msForce = 0;

for n = 1 : length(angularFrequencyVector)
    msAcceleration = msAcceleration + psdAcceleration(n)*deltaAngularFrequency;
    msForce = msForce + psdForce(n)*deltaAngularFrequency;
end

rmsAcceleration = sqrt(msAcceleration);
rmsForce = sqrt(msForce);

figure(100);
semilogy(frequencyVector,psdAcceleration);grid
xlabel('Frequency [Hz]');
ylabel('Acceleration [(m/s^2)^2/Hz]');
title('Sprung mass acceleration PSD');
legend(['rms value = ',num2str(rmsAcceleration)])

figure;
semilogy(frequencyVector,psdForce);grid
xlabel('Frequency [Hz]');
ylabel('Acceleration [(N)^2/Hz]');
title('Tyre force PSD');
legend(['rms value = ',num2str(rmsForce)])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 2.2
%
% Front wheel
rmsAcceleration = zeros(length(frontWheelSuspStiffVector),length(frontWheelSuspDampVector));
rmsForce = zeros(length(frontWheelSuspStiffVector),length(frontWheelSuspDampVector));

for ind1 = 1 : length(frontWheelSuspStiffVector)
    
    for ind2 = 1 : length(frontWheelSuspDampVector)
        
        % Update suspension stiffness and damping
        frontWheelSuspStiff = frontWheelSuspStiffVector(ind1);
        frontWheelSuspDamp = frontWheelSuspDampVector(ind2);
        % Update A and C matrices 
         
        %ADD YOUR CODE HERE
        Af = [0,1,0,0;
            -frontWheelSuspStiff/sprungMassFront, -frontWheelSuspDamp/sprungMassFront, frontWheelSuspStiff/sprungMassFront,frontWheelSuspDamp/sprungMassFront;
            0,0,0,1;
            frontWheelSuspStiff/unsprungMassFront, frontWheelSuspDamp/unsprungMassFront, (-tireStiff-frontWheelSuspStiff)/unsprungMassFront, (-tireDamp-frontWheelSuspDamp)/unsprungMassFront];
               
        % Calculate transfer functions for front wheel Zr to Ride and Tyre force
        
            transferFunctionFrontZrToRide = zeros(length(angularFrequencyVector),1);
            transferFunctionFrontZrToForce = zeros(length(angularFrequencyVector),1);
 
            for j = 1 : length(angularFrequencyVector)
                % Calculate H(w) not the absolut value |H(w)|
                transferFunctionFrontZrToRide(j,:) = -angularFrequencyVector(j)^2*C1f*inv((1i*angularFrequencyVector(j)*eye(4)-Af))*Bf+D1f;
                transferFunctionFrontZrToForce(j,:) = C3f*inv((1i*angularFrequencyVector(j)*eye(4)-Af))*Bf+D3f;
            end
               
        
        % Calculate acceleration and tyre force response spectrum
        psdAcceleration = zeros(length(angularFrequencyVector),1);
        psdForce = zeros(length(angularFrequencyVector),1);
        
        for m = 1 : length(angularFrequencyVector)
            psdAcceleration(m,:) = abs(transferFunctionFrontZrToRide(m))^2*roadSpectrum(m);
            psdForce(m,:) = abs(transferFunctionFrontZrToForce(m))^2*roadSpectrum(m);
        end
        
        % Calculate rms values of acceleration and tyre force
        msAcceleration = 0;
        msForce = 0;
        
        for n = 1 : length(angularFrequencyVector)
            msAcceleration = msAcceleration + psdAcceleration(n)*deltaAngularFrequency;
            msForce = msForce + psdForce(n)*deltaAngularFrequency;
        end
        
        rmsAcceleration(ind1,ind2) = sqrt(msAcceleration);
        rmsForce(ind1,ind2) = sqrt(msForce);
       
    end
end

% Plot rms values vs damping
figure;
plot(frontWheelSuspDampVector,rmsAcceleration);grid
legend(num2str(frontWheelSuspStiffVector'));
xlabel('Dampning coefficient [Ns/m]'); %ADD YOUR CODE HERE
ylabel('RMS Acceleration [m/s^2]'); %ADD YOUR CODE HERE
title('Sprung mass acceleration vs chassis damping for various spring stiffness');
axis([0 10000 0 3]);

figure;
plot(frontWheelSuspDampVector,rmsForce);grid
legend(num2str(frontWheelSuspStiffVector'));
xlabel('Dampning coefficient [Ns/m]'); %ADD YOUR CODE HERE
ylabel('RMS tyre force [N]'); %ADD YOUR CODE HERE
title('Dynamic tyre force vs chassis damping for various spring stiffness');
axis([0 10000 0 1400]);

% Identify optimal damping values for each stiffness value
[optimalRmsAcceleration,iOptimalRmsAcceleration] = min(rmsAcceleration,[],2);
[optimalRmsForce,iOptimalRmsForce] = min(rmsForce,[],2);

% Plot optimal damping for Ride and Tyre force vs Stiffness
figure;
hold on
plot(frontWheelSuspStiffVector,frontWheelSuspDampVector(iOptimalRmsAcceleration)); %ADD YOUR CODE HERE
plot(frontWheelSuspStiffVector,frontWheelSuspDampVector(iOptimalRmsForce)); %ADD YOUR CODE HEREgrid
grid on
xlabel('Spring stiffness [N/m]'); %ADD YOUR CODE HERE
ylabel('Damping coefficient [Ns/m]'); %ADD YOUR CODE HERE
title('Optimal damping vs spring stiffness');
legend('rmsAcce','rmsForce');
axis([10000 65000 0 4000]);

