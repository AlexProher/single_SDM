
A = importdata('../Build/Release/example.txt');
size(A.data)
%%

start_idx = 10;
end_idx = 10000;

X = A.data(start_idx:end_idx,4) - 1.25;
figure;
plot(A.data(1:end_idx,1), A.data(1:end_idx,2))

figure;
plot(A.data(1:end_idx,1), A.data(1:end_idx,4))



T = 0.001;
Fs = 1/T;            % Sampling frequency                        
L = end_idx - start_idx;             % Length of signal
t = (0:L-1)*T;        % Time vector
Y = fft(X);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs/L*(0:(L/2));
figure;
semilogx(f,P1,"LineWidth",3) 
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")