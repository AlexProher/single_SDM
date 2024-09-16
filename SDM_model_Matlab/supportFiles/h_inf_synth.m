function [S, T, SG, KS, K] = h_inf_synth(u_system,P, nmeas, ncon)
%H_ONF_SYNTH Summary of this function goes here
%   Detailed explanation goes here
[K, CL, gamma, info] = hinfsyn(P, nmeas, ncon);
fprintf('H inf gamma for automatic P is %d\n',gamma);

L=series(K, u_system);      % Loop transfer function L=GK 
S=1/(eye(nmeas)+L);         % S=1/(1+L) sensetivity function dy -> y    
T= feedback(L,eye(1));      % T=I-S complementary sens function ref -> y 
SG=S*u_system;              % plant sensitivity funct di -> y
KS=K*S;              % controller sensitivity funct ref -> u
end

