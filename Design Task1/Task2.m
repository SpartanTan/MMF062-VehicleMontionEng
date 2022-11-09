%% 2a
% clc;
clear;
%Local variablels
%Saab_93_datasheet;
syms F_fx F_rx F_fz F_rz wdot_f wdot_r a %Unknown variables
syms m g F_air h R T_f CCx s_fx J l_r l_f RRC s_rx s_fx

%Local variablels
mg = m*g;
wb = l_r+l_f;

%Chassi equations
%Chassi vertical
eq1 = m*a == F_fx + F_rx - F_air;
%Chassi horizontal
eq2 = mg == F_fz + F_rz;
%Moment equib front wheel
eq3 = 0 == -mg*l_f+F_rz*wb-F_air*(h-R);
%Moment equib rear wheel
eq4 = 0 == mg*l_r-F_fz*wb - F_air*(h-R);

%Tyre equations
%   Moment front tyre
eq5 = J*wdot_f == T_f-F_fx*R-F_fz*RRC*R;
%Force horizontal front tyre
eq6 = F_fx == CCx*F_fz*s_fx;
%Moment rear tyre
eq7 = J*wdot_r == -F_rx*R-F_fz*RRC*R;
%Force horizontal rear tyre
eq8 = F_rx == CCx*F_rz*s_rx; %

eqns = [eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq7,eq8];
vars = [F_fx F_rx F_fz F_rz wdot_f wdot_r a];

test = solve(eqns,vars)


