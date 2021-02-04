function [Kc] = Stiffness(Jq,Jt,l_1,l_2)
%STIFFNESS_1 Summary of this function goes here
% INPUTS:
% Jq,Jt - Jacobians w.r.t q and thetas
% l_1,l_2 link lengths(equal for each leg)

% OUTPUTS:
% Kc - stiffness matrix, size = 6x6

%   Detailed explanation goes here% K theta matrix shape =13*13


E = 70e9; %Young's modulus
G = 25.5e9; %shear modulus
d = 50e-3; %link diameter

%for cylinder
S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;

 %3 elements have stiffness(first joint,link1,link2)
k_0 = 1e6; % Actuator stif
k_1 = k_cylinder(E, G, d, l_1, S, Iy, Iz);
k_2 = k_cylinder(E, G, d, l_2, S, Iy, Iz);


Kt = [k_0 zeros(1,12)
    zeros(6,1) k_1 zeros(6,6)
    zeros(6,1) zeros(6,6) k_2];

% just using formulas from KLIMCHIK's presentation
K0_c = inv(Jt*inv(Kt)*Jt');

Kc_q = inv(Jq' * inv(K0_c) * Jq) * Jq' * inv(K0_c);

Kc = K0_c - K0_c * Jq * Kc_q;

end
