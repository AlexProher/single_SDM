clc;
close all;
clear all;
addpath('supportFiles/');
addpath('supportFiles/');
addpath('supportFiles/');

%%
config = readstruct("../sourceFiles/configuration.json");

xBodySize = config.Body.xSize;
yBodySize = config.Body.ySize;
zBodySize = config.Body.zSize;
bodyDensity = config.Body.density_nominal;

Ts = config.General.Ts;

k0 = config.SD.spring_nominal;
c0 = config.SD.damping_nominal;
m0 = xBodySize*yBodySize*zBodySize*bodyDensity;

delta_k = 1;
delta_c = 1;
delta_m = 1;
delta_k = 1-ureal('delta_k', delta_k, 'Percentage', [-50,50]);
delta_c = 1-ureal('delta_c', delta_c, 'Percentage', [-50,50]);
delta_m = 1-ureal('delta_m', delta_m, 'Percentage', [-50,50]);

k = (1+delta_k)*k0;
c = (1+delta_c)*c0;
m = (1+delta_m)*m0;

[A,B,C,D] = SVDModel(m, k, c);
u_system = ss(A,B,C,D);
omega = sqrt(k.NominalValue/m.NominalValue);  % natural frequency

% Ts = 0.001;
sys_d = c2d(u_system.NominalValue,Ts);
sys_wc = c2d(worst_case_tf(u_system), Ts);
%%

u_system.StateName = {'velocity (m/s)';'distance (m)'};
u_system.InputName = {'ui'};
u_system.OutputName = {'x'};

%% Reqirements
OS = 15;
[DampRatio, PM, Mt] = OverShoot(OS);
du = 0.3;
umax = 2*k0;

%% PID

[S_pid, T_pid, SG_pid, KS_pid, PID_tf] = PID_synth_by_Mt(u_system, Mt, PM);

plot_step_response(S_pid, T_pid, SG_pid, KS_pid, 'H inf CL system');
%% Bode diag for Kinf system
w=logspace(-3,3,500); %% to be adjusted
plot_sens_analysis(S_pid,T_pid,SG_pid,KS_pid, w)

PID_d = c2d(PID_tf,Ts);

%% Generalized plant configuration for Hinf synth

sdmeas = sumblk('y1 = do+x+n');    % measurements including output disturbancy
sdact = sumblk('ry = ref-y1');  % reference error definition 
gp_input = sumblk('ui = u+di');  % reference error definition 

%% Weighting function for comlementary sens
% it allows to keep low steady state error
% specify module margin, and system response

tau = stepinfo(u_system.NominalValue).RiseTime/5;
% Mt = OverShoot(15);       % Modulus margin 0.5
eps_t = 1e-1;            % Steady state error 0.001
n = 1;
omega_t = 2.3/tau;     % CL bandwidth, where T cross 0db
Wt = ss((tf([1, omega_t/Mt^(1/n)],[eps_t^(1/n), omega_t]))^n);

Wt.u = 'x' ; Wt.y = 'z3';


%% Weighting function for tracking error
% it allows to keep low steady state error
% specify module margin, and system response

Ms = 2;                     % Modulus margin 0.5
eps_x = 1e-4;               % Steady state error 0.001
omega_x = omega;              % CL bandwidth, where S cross 0db
omega_rx = 1*omega_x;
omega_lx = 0.75*omega_x;

n=1;

We_1 = ss(tf([1/Ms^(1/n), omega_lx],[1,omega_lx*eps_x^(1/n)])^(n)); % _/

We_2 = ss(tf([1,omega_x],[1,20,omega_x*omega_x])); % \/

We_3 = ss(tf([1, omega_rx],1)); %-\
We = We_2*We_1*We_3;

We.u = 'ry' ; We.y = 'z1';

figure;
bode(1/We)
%% Weighting function for controller performance
% Allows to restrict controller action on high freq region
% Restrict restrict saturation

Mks = 0.2*umax;                 % actuator constrains
eps_u = 0.001;                 % noize attenuation
eps_u_2 = 10;
omega_ru = 200*omega;       % cut freq for controller action
omega_lu = 0.001*omega;
n=1;
Wu_1 = ss((tf([1, omega_ru/Mks^(1/n)],[eps_u^(1/n), omega_ru]))^n);
Wu_2 = ss(tf([1/Mks, omega_lu],[1,omega_lu*eps_u_2]));
Wu = Wu_1*Wu_2;

Wu.u = 'u'; Wu.y = 'z2';

figure
bodemag(1/Wu);


%% Generalized plant building for LOWER LFT

ICinputs = {'ref','do', 'di','n', 'u'};     % gen. plant inputs
ICoutputs = {'z1','z2','z3','ry'};   % gen. plant outputs

P = connect(u_system,...
            Wu,We,Wt,...
            sdmeas,sdact,gp_input,...
            ICinputs,ICoutputs);


%% H-inf synth

nmeas = 1;  % number of controller inputs
ncon = 1;   % number of controller outputs

[S_inf, T_inf, SG_inf, KS_inf, K_inf] = h_inf_synth(u_system, P, nmeas, ncon);
plot_step_response(S_inf, T_inf, SG_inf, KS_inf, 'H inf CL system');
%% Bode diag for Kinf system
w=logspace(-3,3,500); %% to be adjusted
plot_sens_analysis_wgth(S_inf,T_inf,SG_inf,KS_inf,We, Wu, Wt, w)

%% UPPER LFT

A = [-c0/m0, -k0/m0;
    1, 0];

B = [-c0/m0, -k0/m0, -1, 1/m0;
    0, 0, 0, 0];

C = [1, 0
    0, 1
    1, 0
    0, 1];

D = 0;

uLFT_system = ss(A,B,C,D);
uLFT_system.u = {'ud1','ud2','ud3', 'ui'};
uLFT_system.y = {'yd1','yd2','yd3', 'x'};

differentiator = ss(tf([1,0],1));
differentiator.u = 'yd3';
differentiator.y = 'dyd3';

Delta = ss([delta_c, 0, 0
            0, delta_k, 0
            0, 0, delta_m]);

Delta.u = {'yd1','yd2','dyd3'};
Delta.y = {'ud1','ud2','ud3'};

ICinputs = {'ui'};     % gen. plant inputs
ICoutputs = {'x'};    % gen. plant outputs

uLFT_system_m = connect(uLFT_system,...
            Delta, differentiator,...
            ICinputs,ICoutputs);

ICinputs = {'ref','di','do','n','u'};     % gen. plant inputs
ICoutputs = {'z1','z2', 'z3','ry'};   % gen. plant outputs


P_unc = connect(uLFT_system_m,...
                Wu, We, Wt,...
                sdmeas, sdact, gp_input,...
                ICinputs, ICoutputs);

%% Mu synth control
opts = musynOptions;
opts.MaxIter = 20;

[S_mu_str, T_mu_str, SG_mu_str, KS_mu_str, Klow_mu_str] = mu_synth(u_system, P_unc, nmeas, ncon, opts, 4);

step_response(S_mu_str, T_mu_str, SG_mu_str, KS_mu_str, 'mu CL system');

%% Bode diag for Mu system
w=logspace(-3,3,500); %% to be adjusted
plot_sens_analysis_wgth(S_mu_str,T_mu_str,SG_mu_str,KS_mu_str,We, Wu, Wt, w)

%%

K_mu_d = c2d(Klow_mu_str, Ts);
K_hinf_d = c2d(K_inf, Ts);

%%
% figure;
% plot(out.tout, out.simout(:,2), "LineStyle","-", LineWidth=2);
% hold on;
% plot(out.tout, out.simout(:,3)-0.2,  "LineStyle","--", LineWidth=2);
% grid on;
% legend("MatLAB model", "CHRONO model")
% fontsize(14, 'points');
% title('Worst Case Gain Model');
% xlabel("Time, s");
% ylabel("Displacement, m");


%%simulations

%%
%%OL
results.ol = out.simout;
results.ol_time = out.tout;

%%
%PID
results.pid = out.simout;
results.pid_time = out.tout;

%%
%Kinf
results.hinf = out.simout;
results.hinf_time = out.tout;

%%
%Mu-syn

results.mu = out.simout;
results.mu_time = out.tout;
%% Time

results.time = out.tout;
%%
save('wc_exp4_noizeHF', "results");
% save('wc_exp4_noize.mat', "results");
%%

% 