clear all
close all
clc

Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector

% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
f1 = 77;
A1 = 0.7;
f2 = 43;
A2 = 2;
S = A1*sin(2*pi*f1*t) + A2*sin(2*pi*f2*t);

% Corrupt the signal with noise 
X = S + 2*randn(size(t));

figure(1);
tiledlayout(1,2)

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
nexttile
plot(1000*t ,X)
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO : Compute the Fourier transform of the signal. 
Y = fft(X);

% TODO : Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);


% Plotting
nexttile
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')