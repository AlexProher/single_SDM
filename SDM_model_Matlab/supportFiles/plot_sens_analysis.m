function [] = plot_sens_analysis(S, T, SG, KS, w)
%PLOT_SENS_ANALYSIS Summary of this function goes here
%   Detailed explanation goes here

figure
subplot(2,2,1), 
bodemag(S,w), title('Sensitivity function')
hold on;
bodemag(S.NominalValue,w)
% bodemag(1/We,w, 'r'), title('Sensitivity function')
legend('Uncertain', 'Nominal');
grid on
subplot(2,2,2),
bodemag(T,w),  title('Complementary sensitivity function')
hold on; 
bodemag(T.NominalValue,w)
% hold on; bodemag(1/Wt,w, 'r'),
legend('Uncertain', 'Nominal');
grid on
subplot(2,2,3), 
bodemag(SG,w), title('Sensitivity*Plant')
hold on; 
bodemag(SG.NominalValue,w)
legend('Uncertain', 'Nominal');
grid on
subplot(2,2,4), bodemag(KS,w), title('Controller*Sensitivity')
hold on; 
bodemag(KS.NominalValue,w)
% hold on; bodemag(1/Wu,w, 'r')
legend('Uncertain', 'Nominal');
grid on

fontsize(14, 'points');
ylabel('Amplitude','FontSize',14)
xlabel('Time','FontSize',14)

Fh = gcf;
Fh.Children(7).Title.String = '';
Fh.Children(10).Title.String = '';
Fh.Children(13).Title.String = '';
Fh.Children(16).Title.String = '';

Fh.Children(7).Children(1).Children.LineWidth = 1;
Fh.Children(10).Children(1).Children.LineWidth = 1;
Fh.Children(13).Children(1).Children.LineWidth = 1;
Fh.Children(16).Children(1).Children.LineWidth = 1;
end

