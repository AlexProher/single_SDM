function [S, T, SG, KS, K, w_i, w_h, w_b, C] = PID_synth_by_Mt(u_system, Mt, PM)

[gain, phase, wout] = bode(u_system.NominalValue); 

PM_rad = 1/Mt;
% PM = 180*PM_rad/pi;
Cmax = abs(1/(PM_rad-1)/max(gain));
[val, idx] = min(abs(gain-1/Cmax));
w_u = wout(idx);

[val, cl_idx] = min(abs(wout-w_u));
arg_G = phase(cl_idx);

cur_PM_deg = -180-arg_G;

a = (1+sind(cur_PM_deg+PM))/(1-sind(cur_PM_deg+PM));
w_b = w_u/sqrt(a);
w_h = w_u*sqrt(a);
K_d = tf([1/w_b, 1],1)*tf(1,[1/w_h, 1]);

[gain_GKd, phase_GKd, wout_GKd] = bode(u_system.NominalValue*K_d); 
[val, idx] = min(abs(wout_GKd-w_u));
arg_GKd = phase_GKd(idx);

cur_PM_deg = -180-arg_GKd;

strange_shift = 10;

w_i = w_u/(tand(cur_PM_deg+90-strange_shift+PM));
K_i = tf([1/w_i, 1],[1/w_i, 0]);
% K_i = 1;
K = K_i*K_d;
[val, cl_idx] = min(abs(wout-w_u));
gain_G = gain(cl_idx);

[gain_K, phase_K, wout_K] = bode(K); 
[val, cl_idx] = min(abs(wout_K-w_u));
gain_K_w_u = gain_K(cl_idx);

C = 1/(gain_G*gain_K_w_u);
K = C*K;

K_ss = ss(K);

L=K_ss*u_system;          % Loop transfer function L=GK 
S=1/(eye(1)+L);             % S=1/(1+L) sensetivity function dy -> y    
T= feedback(L,eye(1));      % T=I-S complementary sens function ref -> y 
SG=S*u_system;              % plant sensitivity funct di -> y
KS=K_ss*S;           % controller sensitivity funct ref -> u

end

