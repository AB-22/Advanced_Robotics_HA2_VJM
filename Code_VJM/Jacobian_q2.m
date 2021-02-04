function [Jq] = Jacobian_q2(T,T_base,T_tool,q,t,l_1,l_2,l_platform)
%JACOBIAN Summary of this function goes here
% Computing jacobian matrix(partial derivatives of FK for each axis w.r.t. parameters q) 
% Jacobians are computed numerically


%   Detailed explanation goes here
% INPUTS:
% T_base - transformation from global frame to local frame of leg
% T_tool - transformation from last joint to end effector
% q - transformations for each joint [translation,rotation,rotation,rotation] to reach end effector position
% t - thetas(all zero)
% l_1,l_2 link lengths(equal for each leg)

% OUTPUTS:
% Jacobian matrix with size = 6 x number_of_parameters for leg 'z'


T_0 = T; 
T_0(1:3,4) = [0;0;0];
T_0 = T_0';

%first column for jacobian_q

Td = T_base * Tz(q(1)) * Tz(t(1)) * Rzd(q(2)) * Tx((l_1))* Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Rz(q(3)) * Tx((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13)) * Rz(q(4))*Tx(-l_platform)  * T_tool*T_0;

Jq_1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

%second column for jacobian_q
Td = T_base * Tz(q(1)) * Tz(t(1)) * Rz(q(2)) * Tx((l_1)) * Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Rzd(q(3)) * Tx((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13)) * Rz(q(4))*Tx(-l_platform) * T_tool*T_0;

Jq_2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

%second column for jacobian_q
Td = T_base * Tz(q(1)) * Tz(t(1)) * Rz(q(2)) * Tx((l_1))* Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Rz(q(3)) * Tx((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13)) * Rzd(q(4))*Tx(-l_platform) * T_tool*T_0;

Jq_3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

Jq = [Jq_1, Jq_2, Jq_3];
end
