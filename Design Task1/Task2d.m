%% 2a
% clc;
clear;
%Local variablels
%Saab_93_datasheet;
syms F_fx F_rx F_fz F_rz wdot_f wdot_r a %Unknown variables
syms M g Fair h R T_f  s_fx J L_r L_f f_r s_rx s_fx theta mu_f mu_r

%Local variablels
wb = L_r+L_f;

%Chassi equations
%Chassi vertical
eq1 = M*a == F_fx + F_rx - Fair - M*g*sin(theta)
%Chassi horizontal
eq2 = M*g*cos(theta) == F_fz + F_rz;
%Moment front wheel
eq3 = 0 == -M*g*cos(theta)*L_f+F_rz*wb-Fair*(h-R);
%Moment rear wheel
eq4 = 0 == M*g*cos(theta)*L_r-F_fz*wb - Fair*(h-R);

%Tyre equations
%   Moment front tyre
eq5 = J*wdot_f == T_f-F_fx*R-F_fz*f_r*R;
%Force horizontal front tyre
eq6 = F_fx == F_fz*mu_f;
%Moment rear tyre
eq7 = J*wdot_r == -F_rx*R-F_fz*f_r*R;
%Force horizontal rear tyre
eq8 = F_rx == F_rz*mu_r; %

eqns = [eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq7,eq8];
vars = [F_fx F_rx F_fz F_rz wdot_f wdot_r a];

solv = solve(eqns,vars)


