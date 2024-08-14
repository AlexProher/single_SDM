%%
% load('nominal_slow.mat', 'results');
load('worstCase_slow.mat', 'results');

%%
figure;
subplot(3,1,1);

hold on;
plot(results.pid_time, results.pid(:,1)-1.2,  "LineStyle","--", LineWidth=2);
plot(results.hinf_time, results.hinf(:,1)-1.2,  "LineStyle","-.", LineWidth=2);
plot(results.mu_time, results.mu(:,1)-1.2,  "LineStyle",":", LineWidth=2);
plot(results.ol_time, results.ol(:,1)-1.2, "LineStyle","-", LineWidth=2);

grid on;
legend("PID", "Hinf", "mu-synth", "openLoop")
fontsize(14, 'points');
% title("Nomianl Model Wheel Vertical Displacement")
title("WorstCase Model Wheel Vertical Displacement")
xlabel("Time, s");
ylabel("Displacement, m");
xlim([2,17]);

subplot(3,1,2);

plot(results.pid_time, results.pid(:,2),  "LineStyle","--", LineWidth=2);
hold on;
plot(results.hinf_time, results.hinf(:,2),  "LineStyle","-.", LineWidth=2);
plot(results.mu_time, results.mu(:,2),  "LineStyle",":", LineWidth=2);
plot(results.ol_time, results.ol(:,2)-0.2, "LineStyle","-", LineWidth=2);
grid on;
% legend("PID", "Hinf", "mu-synth", "OL")
fontsize(14, 'points');
title("WorstCase Model Body Vertical Displacement")
% title("Nomianl Model Body Vertical Displacement")
xlabel("Time, s");
ylabel("Displacement, m");
xlim([2,17]);

subplot(3,1,3);

plot(results.pid_time, results.pid(:,3)+10,  "LineStyle","--", LineWidth=2);
hold on;
plot(results.hinf_time, results.hinf(:,3)+10,  "LineStyle","-.", LineWidth=2);
plot(results.mu_time, results.mu(:,3)+10,  "LineStyle",":", LineWidth=2);
grid on;
% legend("PID", "Hinf", "mu-synth")
fontsize(14, 'points');
title("WorstCase Model Controller Out")
% title("Nomianl Wheel Controller Out")
xlabel("Time, s");
ylabel("Force, N");
xlim([2,17]);

%%
metrics = struct();
ce.pid = trapz(results.pid_time, (results.pid(:, 3)+20).^2);
ce.hinf = trapz(results.hinf_time, (results.hinf(:, 3)+20).^2);
ce.mu = trapz(results.mu_time, (results.mu(:, 3)+20).^2);

iae.pid = sum(abs(results.pid(:,2)));
iae.hinf = sum(abs(results.hinf(:,2)));
iae.mu = sum(abs(results.mu(:,2)));