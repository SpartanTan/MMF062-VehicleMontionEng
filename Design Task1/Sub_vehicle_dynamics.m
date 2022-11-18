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
Fzf = (L_r*M*g*cos(theta) - M*g*h*mu_r*cos(theta))/(L_f + L_r + h*mu_f - h*mu_r);    %[N] front normal force
Fzr = (L_f*M*g*cos(theta) + M*g*h*mu_f*cos(theta))/(L_f + L_r + h*mu_f - h*mu_r);  %[N] rear normal force
omegaDotf = (L_f*T_f + L_r*T_f + T_f*h*mu_f - T_f*h*mu_r - L_r*M*R*f_r*g*cos(theta) - L_r*M*R*g*mu_f*cos(theta) + M*R*f_r*g*h*mu_r*cos(theta) + M*R*g*h*mu_f*mu_r*cos(theta))/(J*(L_f + L_r + h*mu_f - h*mu_r)); %[rad/s/s] rotational acceleration, front
omegaDotr = (L_f*T_r + L_r*T_r + T_r*h*mu_f - T_r*h*mu_r - L_r*M*R*f_r*g*cos(theta) - L_f*M*R*g*mu_r*cos(theta) + M*R*f_r*g*h*mu_r*cos(theta) - M*R*g*h*mu_f*mu_r*cos(theta))/(J*(L_f + L_r + h*mu_f - h*mu_r)); %[rad/s/s] rotational acceleration, rear
vDot = -(Fair*L_f + Fair*L_r + Fair*h*mu_f - Fair*h*mu_r + L_f*M*g*sin(theta) + L_r*M*g*sin(theta) - L_f*M*g*mu_r*cos(theta) - L_r*M*g*mu_f*cos(theta) + M*g*h*mu_f*sin(theta) - M*g*h*mu_r*sin(theta))/(M*(L_f + L_r + h*mu_f - h*mu_r));     %[m/s/s] acceleration

% Write the explicit solutions here!
% Fzf = subs(solv.F_fz,theta);    %[N] front normal force
% Fzr = sym2poly(solv.F_rz,theta);  %[N] rear normal force
% omegaDotf = sym2poly(solv.wdot_f,[theta,]);
% omegaDotr = sym2poly(solv.wdot_r);% 
% vDot = sym2poly(solv.a);
end