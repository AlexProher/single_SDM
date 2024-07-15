clc;
close all;
clear all;
addpath('C:\Users\Aleksandr Prokohrov\Documents\ProkhorovAlex\UGA_INP\internship\MatLabProjects/functions/');
addpath('C:\Users\Aleksandr Prokohrov\Documents\ProkhorovAlex\UGA_INP\internship\MatLabProjects/synthesis_functions/');
addpath('C:\Users\Aleksandr Prokohrov\Documents\ProkhorovAlex\UGA_INP\internship\MatLabProjects/plot_figures_files/');

%%
config = readstruct("../sourceFiles/configuration.json");

xBodySize = config.Body.xSize;
yBodySize = config.Body.ySize;
zBodySize = config.Body.zSize;
bodyDensity = config.Body.density;

k = config.SD.spring;
c = config.SD.damping;
m = xBodySize*yBodySize*zBodySize*bodyDensity;


[A,B,C,D] = SVDModel_MISO(m, k, c);
system_miso = ss(A,B,C,D);

[A,B,C,D] = SVDModel(m, k, c);
system_siso = ss(A,B,C,D);


Ts = 0.001;
sys_d = c2d(system_miso,Ts);

%%

system_siso.StateName = {'velocity (m/s)';'distance (m)'};
system_siso.InputName = {'ui'};
system_siso.OutputName = {'x'};

%% Reqirements
OS = 15;
[DampRatio, PM, Mt] = OverShoot(OS);
du = 0.1;
umax = 4*m*9.8;
bmax = 0.1;

%% PID
% [S_pid, T_pid, SG_pid, KS_pid, PID_tf] = PID_synth(uss(system_siso), PM, du, umax, bmax);
[S_pid, T_pid, SG_pid, KS_pid, PID_tf] = PID_synth(uss(system_siso), 5, 8, 120, 230);
plot_step_response(S_pid, T_pid, SG_pid, KS_pid, 'PID CL system');

PID_d = c2d(PID_tf,Ts);

