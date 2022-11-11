function  [vDot,omegaDotf,omegaDotr,Fzf,Fzr]=...
    Sub_vehicle_dynamics(v,Tdrivf,Tdrivr,slipf,slipr,slope,road_cond,CONST)

% Read parameters
c_d=CONST.c_d;
A_f=CONST.A_f;
air=CONST.air;

M=CONST.M;
Ir=CONST.Ir; %[kg*m^2] Moment of inertia for two wheels and axle 
g=CONST.g;

h=CONST.h;
L=CONST.L;
L_f=CONST.Lf;
L_r=CONST.Lr;

R=CONST.R;
f_r=CONST.f_r; % rolling resistance coefficient

% Calculate air resistance 
Fair=0.5*c_d*A_f*air*v^2;

% Magic Tire Formula
muf=Sub_magic_tireformula(slipf,road_cond);
mur=Sub_magic_tireformula(slipr,road_cond);

% Write the explicit solutions here!

% %
% Theta -> slope
% Mu_r -> mur
% Mu_f -> muf
% TT_f -> Tdrivf
% TT_r -> Tdrivr
% JJ -> Ir
% FF_air -> Fair

theta = slope;
mu_r = mur;
mu_f = muf;
T_f = Tdrivf;
T_r = Tdrivr;
J = Ir;
F_air = Fair;

% Fzf = (L_r*M*g*cos(slope) - M*R*g*sin(slope) + M*g*h*sin(slope) + M*R*g*mur*cos(slope) - M*g*h*mur*cos(slope))/(L_f + L_r - R*muf + R*mur + h*muf - h*mur);    %[N] front normal force
% Fzr = (L_f*M*g*cos(slope) + M*R*g*sin(slope) - M*g*h*sin(slope) - M*R*g*muf*cos(slope) + M*g*h*muf*cos(slope))/(L_f + L_r - R*muf + R*mur + h*muf - h*mur);    %[N] rear normal force
% omegaDotf = (L_f*Tdrivf + L_r*Tdrivf - R*Tdrivf*muf + R*Tdrivf*mur + Tdrivf*h*muf - Tdrivf*h*mur + M*R^2*f_r*g*sin(slope) + M*R^2*g*muf*sin(slope) - M*R*f_r*g*h*sin(slope) - M*R*g*h*muf*sin(slope) - M*R^2*f_r*g*mur*cos(slope) - M*R^2*g*muf*mur*cos(slope) - L_r*M*R*f_r*g*cos(slope) - L_r*M*R*g*muf*cos(slope) + M*R*f_r*g*h*mur*cos(slope) + M*R*g*h*muf*mur*cos(slope))/(Ir*(L_f + L_r - R*muf + R*mur + h*muf - h*mur)); %[rad/s/s] rotational acceleration, front
% omegaDotr = (L_f*Tdrivr + L_r*Tdrivr - R*Tdrivr*muf + R*Tdrivr*mur + Tdrivr*h*muf - Tdrivr*h*mur - M*R^2*f_r*g*sin(slope) - M*R^2*g*mur*sin(slope) + M*R*f_r*g*h*sin(slope) + M*R*g*h*mur*sin(slope) + M*R^2*f_r*g*muf*cos(slope) + M*R^2*g*muf*mur*cos(slope) - L_f*M*R*f_r*g*cos(slope) - L_f*M*R*g*mur*cos(slope) - M*R*f_r*g*h*muf*cos(slope) - M*R*g*h*muf*mur*cos(slope))/(Ir*(L_f + L_r - R*muf + R*mur + h*muf - h*mur)); %[rad/s/s] rotational acceleration, rear
% vDot = -(Fair*L_f + Fair*L_r - Fair*R*muf + Fair*R*mur + Fair*h*muf - Fair*h*mur + L_f*M*g*sin(slope) + L_r*M*g*sin(slope) - L_f*M*g*mur*cos(slope) - L_r*M*g*muf*cos(slope))/(M*(L_f + L_r - R*muf + R*mur + h*muf - h*mur));%[m/s/s] acceleration

Fzf = (L_r*M*g*cos(slope) - M*g*h*mur*cos(slope))/(L_f + L_r + h*muf - h*mur);
Fzr = (L_f*M*g*cos(slope) + M*g*h*muf*cos(slope))/(L_f + L_r + h*muf - h*mur);
omegaDotf = (L_f*Tdrivf + L_r*Tdrivf + Tdrivf*h*muf - Tdrivf*h*mur - L_r*M*R*f_r*g*cos(slope) - L_r*M*R*g*muf*cos(slope) + M*R*f_r*g*h*mur*cos(slope) + M*R*g*h*muf*mur*cos(slope))/(Ir*(L_f + L_r + h*muf - h*mur));
omegaDotr = -(Tdrivr*h*mur - L_r*Tdrivr - Tdrivr*h*muf - L_f*Tdrivr + L_f*M*R*f_r*g*cos(slope) + L_f*M*R*g*mur*cos(slope) + M*R*f_r*g*h*muf*cos(slope) + M*R*g*h*muf*mur*cos(slope))/(Ir*(L_f + L_r + h*muf - h*mur));
vDot = -(Fair*L_f + Fair*L_r + Fair*h*muf - Fair*h*mur + L_f*M*g*sin(slope) + L_r*M*g*sin(slope) - L_f*M*g*mur*cos(slope) - L_r*M*g*muf*cos(slope) + M*g*h*muf*sin(slope) - M*g*h*mur*sin(slope))/(M*(L_f + L_r + h*muf - h*mur));

end