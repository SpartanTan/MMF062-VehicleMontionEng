%% Setup the car
clc;
clear;
CAR.m = 1675; %Mass [kg]
CAR.L = 2.675; % distance [m]
CAR.g = 9.81; %Gravity constant
CAR.steeringRatio = 15.9;


%% 1.2: Understeer gradient (0.5 point)
lf1 = 0.37;
lf2 = 0.63;
lf3 = 0.47;


%Case 1
Ku1 = CalcKu(lf1,CAR)
%Case 2
Ku2 = CalcKu(lf2,CAR)
%Case 3
Ku3 = CalcKu(lf3,CAR)

%% Task 1.3: Critical/Characteristic speed (0.5 point)
%For all load cases above, determine the critical and characteristic speeds 

VxChar1 = GetCharSpeed(Ku1,CAR)
VxCrit2 = GetCriticalSpeed(Ku2,CAR)
VxChar3 = GetCharSpeed(Ku3,CAR)
%% Task 1.4: Steering wheel angle for one certain operating point (1 point)
%For all load cases above, determine the steering wheel angle required to get
% 4 m/s2 of lateral acceleration at 100 km/h

ay = 4; % [m/s^2]
vx = 100/3.6;%[m/s] convert 100 km/h to m/s
df1 = ay*(CAR.L+Ku1*CAR.m*vx^2)/vx^2
df2 = ay*(CAR.L+Ku2*CAR.m*vx^2)/vx^2
df3 = ay*(CAR.L+Ku3*CAR.m*vx^2)/vx^2

%% Task 1.5 Steering wheel angle for varying operating point (1 point)
%Plot the steering wheel angle required vs speed curves for the three
% cases for a curve radius of 200 m. Which of the load cases is 
% understeer, oversteer and (relatively) neutral steer?
clf;
clc;
Rp = 200;%[m] radius of corner
vx = 0:0.1:100;
df1 = GetSteeringAngle(vx,Rp,Ku1,CAR);
df2 = GetSteeringAngle(vx,Rp,Ku2,CAR);
df3 = GetSteeringAngle(vx,Rp,Ku3,CAR);
hold on
plot(vx,df1);
plot(vx,df2);
plot(vx,df3);
legend('Ku1', 'Ku2','Ku3');


%% Functions
function vxCrit = GetCriticalSpeed(Ku,CAR)
    vxCrit = sqrt(CAR.L/(-Ku*CAR.m));
end

function vxChar = GetCharSpeed(Ku,CAR)
    vxChar = sqrt(CAR.L/(Ku*CAR.m));
end
function df = GetSteeringAngle(vx,Rp,Ku,CAR)
    df = (CAR.L/Rp + Ku*CAR.m.*vx.^2/Rp)*CAR.steeringRatio;
end
function Ku = CalcKu(lf1,CAR)
c0 = 30.7;
c1 = -0.00235;

F_fz1 = CAR.m*CAR.g*(1-lf1)/2; %Per wheel
F_rz1 = CAR.m*CAR.g*lf1/2; %Per wheel
L_f1 = CAR.L*lf1;
L_r1 = CAR.L*(1-lf1);

Cf1 = 2*(c0*F_fz1+c1*F_fz1^2);%Per axle
Cr1 = 2*(c0*F_rz1+c1*F_rz1^2);%Per axle 
Ku = (Cr1*L_r1-Cf1*L_f1)/(Cf1*Cr1*CAR.L);
end

