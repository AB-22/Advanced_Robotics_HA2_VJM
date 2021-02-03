function [Jt] = Jacobian_t3(T,T_base,T_tool,q,t,l_1,l_2,l_platform)
%JACOBIAN Summary of this function goes here
% Computing jacobian matrix(partial derivatives of FK for each axis w.r.t. parameters theta) 
% Jacobians are computed numerically 


%   Detailed explanation goes here
% INPUTS:
% T_base - transformation from global frame to local frame of leg
% T_tool - transformation from last joint to end effector
% q - transformations for each joint [translation,rotation,rotation,rotation] to reach end effector position
% t - thetas(all zero)
% l_1,l_2 link lengths(equal for each leg)

% OUTPUTS:
% Jacobian matrix with size = 6 x number_of_parameters for leg 'y'

T_0 = T; 
T_0(1:3,4) = [0;0;0];
T_0 = T_0';

% first
Td = T_base * Ty(q(1)) * Tyd(t(1)) * Ry(q(2)) * Tz((l_1)) * Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% second
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Txd(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) *Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% third
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Tx(t(2)) * Tyd(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) *Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% forth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Tx(t(2)) * Ty(t(3)) * Tzd(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) *Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% fifth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rxd(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) *Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_5 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% sixth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ryd(t(6)) * Rz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_6 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% seventh
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rzd(t(7))...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_7 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% eighth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Txd(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_8 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% nineth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Tyd(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_9 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% tenth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) *Tx(t(8)) * Ty(t(9)) * Tzd(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_10 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% eleventh
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rxd(t(11)) * Ry(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_11 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% twelth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ryd(t(12)) * Rz(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_12 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

% thirteenth
Td = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz((l_1)) * Rx(t(2)) * Ry(t(3)) * Rz(t(4)) * Tx(t(5)) * Ty(t(6)) * Tz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rzd(t(13))*Ry(q(4))*Tz(l_platform) *T_tool*T_0;

Jt_13 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';

Jt = [Jt_1, Jt_2, Jt_3, Jt_4,Jt_5, Jt_6, Jt_7, Jt_8,Jt_9, Jt_10, Jt_11, Jt_12,Jt_13];
end

