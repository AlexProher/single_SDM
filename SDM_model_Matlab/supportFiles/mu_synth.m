function [S, T, SG, KS, Klow] = mu_synth(u_system,P, nmeas, ncon, opts, min_order)
%MU_SYNTH Summary of this function goes here
%   Detailed explanation goes here
[Kmu, CLmu, info] = musyn(P, nmeas, ncon, opts);

% Klow = reduceOrder(Kmu, P, CLmu, min_order);
Klow = Kmu;

L=series(Klow, u_system);    % Loop transfer function L=GK 
S=1/(eye(1)+L);             % S=1/(1+L) sensetivity function dy -> y 
T= feedback(L,eye(1));      % T=I-S complementary sens function ref -> y 
SG=S*u_system;              % plant sensitivity funct di -> y
KS=Klow*S;           % controller sensitivity funct ref -> u

end

