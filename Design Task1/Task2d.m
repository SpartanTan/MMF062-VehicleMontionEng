%% 2a
% clc;
%Local variablels
%Saab_93_datasheet;
syms F_fx F_rx F_fz F_rz wdot_f wdot_r a %Unknown variables
syms M g Fair h R T_f  s_fx J L_r L_f f_r s_rx s_fx theta mu_f mu_r T_r h_air

%Local variablels
wb = L_r+L_f;

%Chassi equations
%Chassi vertical
%eq1 = M*g*cos(theta) == F_fz + F_rz;
%Chassi horizontal
eq2 = M*a == F_fx + F_rx - Fair - M*g*sin(theta);

% %Moment front wheel
eq3 = 0 == F_rz*wb-M*g*cos(theta)*L_f-M*g*sin(theta)*h-M*a*h-Fair*h_air;
%Moment rear axl
eq4 = 0 == -F_fz*wb+M*g*cos(theta)*L_r-M*g*sin(theta)*h-M*a*h-Fair*h_air;

%Tyre equations
%   Moment front tyre
eq5 = J*wdot_f == T_f-F_fx*R-F_fz*f_r*R;
%Force horizontal front tyre
eq6 = F_fx == F_fz*mu_f;
%Moment rear tyre
eq7 = J*wdot_r == T_r-F_rx*R-F_fz*f_r*R;
%Force horizontal rear tyre
eq8 = F_rx == F_rz*mu_r; %

eqns = [eq1,eq2,eq4,eq5,eq6,eq7,eq8];
vars = [F_fx F_rx F_fz F_rz wdot_f wdot_r a];

solv = solve(eqns,vars);

