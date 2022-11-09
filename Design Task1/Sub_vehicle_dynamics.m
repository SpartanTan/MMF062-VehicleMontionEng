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
%Personal variables convertion
theta = slope;
T_f = Tdrivf;
T_r = Tdrivr;
mu_f = muf;
mu_r = mur;
J = Ir;

% Write the explicit solutions here!
Fzf = (Fair*R - Fair*h + L_r*M*g*cos(theta))/(L_f + L_r);    %[N] front normal force
Fzr = (Fair*h - Fair*R + L_f*M*g*cos(theta))/(L_f + L_r);   %[N] rear normal force
omegaDotf = (L_f*T_f + L_r*T_f - Fair*R^2*f_r - Fair*R^2*mu_f + Fair*R*f_r*h + Fair*R*h*mu_f - L_r*M*R*f_r*g*cos(theta) - L_r*M*R*g*mu_f*cos(theta))/(J*(L_f + L_r)); %[rad/s/s] rotational acceleration, front
omegaDotr = (L_f*T_r + L_r*T_r - Fair*R^2*f_r + Fair*R^2*mu_r + Fair*R*f_r*h - Fair*R*h*mu_r - L_r*M*R*f_r*g*cos(theta) - L_f*M*R*g*mu_r*cos(theta))/(J*(L_f + L_r)); %[rad/s/s] rotational acceleration, rear
vDot = -(Fair*L_f + Fair*L_r - Fair*R*mu_f + Fair*R*mu_r + Fair*h*mu_f - Fair*h*mu_r + L_f*M*g*sin(theta) + L_r*M*g*sin(theta) - L_f*M*g*mu_r*cos(theta) - L_r*M*g*mu_f*cos(theta))/(M*(L_f + L_r));     %[m/s/s] acceleration

end