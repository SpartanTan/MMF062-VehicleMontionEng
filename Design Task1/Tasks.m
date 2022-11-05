%% 1a
clc;
clear;
clf;
%Dry Asphalt
D = 1;
C = 1.45;
E = -4;
resultDry = CalcFxFz(D,C,E,0.01);
hold on
plot(resultDry(:,1),resultDry(:,2));
%Wet Asphalt
D = 0.6;
C = 1.35;
E = -0.2;
resultWet = CalcFxFz(D,C,E,0.01);
hold on
plot(resultWet(:,1),resultWet(:,2));
%Ice
D = 0.1;
C = 1.55;
E = 0.8;
resultIce = CalcFxFz(D,C,E,0.01);
hold on
plot(resultIce(:,1),resultIce(:,2));
legend('Dry Asphalt', 'Wet Asphalt', 'Ice')

%% 1b
clc;

%Plot orignal data
Data1 = [0:0.05:0.95;   0,0.25,0.53,0.77,0.89,0.95,0.94,0.92,0.9,0.86,0.85,0.83,0.81,0.80,0.79,.78,.77,.76,.75,.74];
subplot(2,1,1)
plot(Data1(1,:),Data1(2,:))
%Create new data
D = 0.952;
C = 1.5;
E = -4; % -0.5<= E <= -4
subplot(2,1,2)
Test1 = CalcFxFz(D,C,E,0.05);
plot(Test1(:,1),Test1(:,2));







function [result] = CalcFxFz(D,C,E,stepLength)
    K = 3*pi/180;
    B = 100*atan(K)/(C*D);
    
    sx= 0;
    FxFz = D*sin(C*atan(B*sx-E*(B*sx-atan(B*sx))));
    result = [sx,FxFz];

    for sx=stepLength:stepLength:1
        FxFz = D*sin(C*atan(B*sx-E*(B*sx-atan(B*sx))));
        result = [result;sx,FxFz];
    end
end
