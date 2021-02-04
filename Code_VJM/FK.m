function [T] = FK(T_base,T_tool,q,t,l_1,l_2,l_platform,leg)
%FK_1 Summary of this function goes here
% Calculating forward kinematics for given leg

%   Detailed explanation goes here
% INPUTS:
% T_base - transformation from global frame to local frame of leg
% T_tool - transformation from last joint to end effector
% q - transformations for each joint [translation,rotation,rotation,rotation] to reach end effector position
% t - thetas(all zero)
% l_1,l_2 link lengths(equal for each leg)
% leg - leg of robot tripteron 'x' or 'y' or 'z'

% OUTPUTS:
% transformation matrix

% We have for leg z for example the following transformations
% active joint + 1 DOF virtual spring + passive joint + rigid link + 6 DOF
% virtual spring + passive joint + rigid link + 6DOF virtual spring + passive joint + Rigid platform

%We can model Rigid platform as a rigid link for each leg

%     here it's just multiplication of homogeneous matricesm, each of them has size4x4
if leg=='x'
    Tplatform=Tx(-l_platform*cosd(60))*Ty(l_platform*sind(60));
    T = T_base * Tx(q(1)) * Tx(t(1)) * Rx(q(2)) * Ty(l_1)* Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7))  ...
    *Rx(q(3)) * Ty(l_2) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13)) * Rx(q(4))*Tplatform * T_tool;
end

if leg=='y'
    Tplatform=Tx(-l_platform*cosd(60))*Ty(-l_platform*sind(60));
    T = T_base * Ty(q(1)) * Ty(t(1)) * Ry(q(2)) * Tz(l_1) * Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7)) ...
    *Ry(q(3)) * Tz((l_2)) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13)) * Ry(q(4))*Tplatform *T_tool;
end

if leg =='z'
    T = T_base * Tz(q(1)) * Tz(t(1)) * Rz(q(2)) * Tx(l_1)* Tx(t(2)) * Ty(t(3)) * Tz(t(4)) * Rx(t(5)) * Ry(t(6)) * Rz(t(7))  ...
    *Rz(q(3)) * Tx(l_2) * Tx(t(8)) * Ty(t(9)) * Tz(t(10)) * Rx(t(11)) * Ry(t(12)) * Rz(t(13)) * Rz(q(4))*Tx(-l_platform) * T_tool;
end


end

