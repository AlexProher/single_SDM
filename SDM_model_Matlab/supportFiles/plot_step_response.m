function [] = step_response(S, T, SG, KS, fig_title)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

figure('Name',fig_title)
subplot(1,3,1), 
step(gridureal(S, 50)); hold on; grid on;
step(S.NominalValue, 'r');
legend('Uncertain', 'Nominal')
% title('Out dist rejection');
title('');

subplot(1,3,2), step(gridureal(T, 50)); hold on; grid on;
step(T.NominalValue, 'r');
legend('Uncertain', 'Nominal')
% title('Ref tracking');
title('');

subplot(1,3,3), step(gridureal(SG, 50)); hold on; grid on;
step(SG.NominalValue, 'r');
legend('Uncertain', 'Nominal')
% title('Input Disturbance rejection');
title('');

fontsize(14, 'points');
ylabel('Amplitude','FontSize',14)
xlabel('Time','FontSize',14)

Fh = gcf;
Fh.Children(5).Title.String = '';
Fh.Children(7).Title.String = '';
Fh.Children(9).Title.String = '';

Fh.Children(5).Children(1).Children(2).LineWidth = 1;
Fh.Children(7).Children(1).Children(2).LineWidth = 1;
Fh.Children(9).Children(1).Children(2).LineWidth = 1;


end

