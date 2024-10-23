%%
% load('nominal_exp4_noizeHF.mat', 'results');
load('wc_exp4_noizeHF.mat', 'results');

%%
figure;
subplot(3,1,1);

hold on;
plot(results.pid_time, results.pid(:,1)-1.1,  "LineStyle","--", LineWidth=2);
plot(results.hinf_time, results.hinf(:,1)-1.1,  "LineStyle","-.", LineWidth=2);
plot(results.mu_time, results.mu(:,1)-1.1,  "LineStyle",":", LineWidth=2);
plot(results.ol_time, results.ol(:,1)-1.1, "LineStyle","-", LineWidth=2);

grid on;
legend("PID", "Hinf", "mu-synth", "openLoop")
fontsize(14, 'points');
% title("Nomianl Model Wheel Vertical Displacement")
title("WorstCase Model Wheel Vertical Displacement")
xlabel("Time, s");
ylabel("Displacement, m");
xlim([1,10]);

subplot(3,1,2);

plot(results.pid_time, results.pid(:,2),  "LineStyle","--", LineWidth=2);
hold on;
plot(results.hinf_time, results.hinf(:,2),  "LineStyle","-.", LineWidth=2);
plot(results.mu_time, results.mu(:,2),  "LineStyle",":", LineWidth=2);
plot(results.ol_time, results.ol(:,2)-0.1, "LineStyle","-", LineWidth=2);
grid on;
% legend("PID", "Hinf", "mu-synth", "OL")
fontsize(14, 'points');
title("WorstCase Model Body Vertical Displacement")
% title("Nomianl Model Body Vertical Displacement")
xlabel("Time, s");
ylabel("Displacement, m");
xlim([1,10]);

subplot(3,1,3);

plot(results.pid_time, results.pid(:,3)+250,  "LineStyle","--", LineWidth=2);
hold on;
plot(results.hinf_time, results.hinf(:,3)+250,  "LineStyle","-.", LineWidth=2);
plot(results.mu_time, results.mu(:,3)+250,  "LineStyle",":", LineWidth=2);
grid on;
% legend("PID", "Hinf", "mu-synth")
fontsize(14, 'points');
title("WorstCase Model Controller Out")
% title("Nomianl Model Controller Out")
xlabel("Time, s");
ylabel("Force, N");
xlim([1,10]);

%%
metrics = struct();
ce.pid = trapz(results.pid_time, (results.pid(:, 3)+500).^2);
ce.hinf = trapz(results.hinf_time, (results.hinf(:, 3)+500).^2);
ce.mu = trapz(results.mu_time, (results.mu(:, 3)+500).^2);

iae.pid = sum(abs(results.pid(:,2)));
iae.hinf = sum(abs(results.hinf(:,2)));
iae.mu = sum(abs(results.mu(:,2)));

%%


figure;
% plot(results.ol_time, results.mu(:,1)-1.1)




T = 0.001;
Fs = 1/T;            % Sampling frequency                        
L = size(results.pid_time,1);             % Length of signal
t = (0:L-1)*T;        % Time vector
X = results.mu(:,2);
Y = fft(X);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = 2*pi*Fs/L*(0:(L/2));
% figure;

semilogx(f,P1,"LineWidth",3) 
title("Output signal amplitude spectrum")
xlabel("Frequency (rad/s)")
ylabel("|P1(f)|")
grid on
hold on;
legend("OL", "PID", "Hinf", "mu")
fontsize(14, 'points');