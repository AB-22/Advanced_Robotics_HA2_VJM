function [Kc] = Virtual_joint(T,T_base,T_tool,q,t,l_1,l_2,l_platform,leg)
%VIRTUAL_JOINT Summary of this function goes here
% Calculating stiffness matrix 

%   Detailed explanation goes here
% INPUTS:
% T_base - transformation from global frame to local frame of leg
% T_tool - transformation from last joint to end effector
% q - transformations for each joint [translation,rotation,rotation,rotation] to reach end effector position
% t - thetas(all zero)
% l_1,l_2 link lengths(equal for each leg)
% leg - leg of robot tripteron 'x' or 'y' or 'z'

% OUTPUTS:
% Kc - stiffness matrix (size = 6x6)


if leg =='x'
    
%     numerically computed Jacobian for leg 'x' w.r.t. parameters 'q'
    Jq = Jacobian_q1(T,T_base,T_tool,q,t,l_1,l_2,l_platform);

%     numerically computed Jacobian for leg 'x' w.r.t. parameters 'theta'
    Jt = Jacobian_t1(T,T_base,T_tool,q,t,l_1,l_2,l_platform);

    % Here We find analytical solution for Kc=K0_c - K0_c*J_q*Kc_q
    Kc = Stiffness(Jq,Jt,l_1,l_2);
    
end

if leg=='z'
    Jq = Jacobian_q2(T,T_base,T_tool,q,t,l_1,l_2,l_platform);

    % Here we can use the same strange math to obtain Jacobian_theta
    Jt = Jacobian_t2(T,T_base,T_tool,q,t,l_1,l_2,l_platform);

    % Here i find analytical solution for Kc=K0_c - K0_c*J_q*Kc_q
    Kc = Stiffness(Jq,Jt,l_1,l_2);
end

if leg =='y'
    Jq = Jacobian_q3(T,T_base,T_tool,q,t,l_1,l_2,l_platform);

    % Here we can use the same strange math to obtain Jacobian_theta
    Jt = Jacobian_t3(T,T_base,T_tool,q,t,l_1,l_2,l_platform);

    % Here i find analytical solution for Kc=K0_c - K0_c*J_q*Kc_q
    Kc = Stiffness(Jq,Jt,l_1,l_2);

    
end