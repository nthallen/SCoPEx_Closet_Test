%% ************************************************************
% Basic FFT example from MATLAB help

% Define conditions
f1 = 90;          % Signal Component 1 Freq        (Hz)
f2 = 420;         % Signal Component 2 Freq        (Hz)
A1 = 0.5;         % Signal Component 1 Amplitude   (au)
A2 = 2.0;         % Signal Component 2 Amplitude   (au)
A3 = 1.0;         % rms value of random noise      (au)

Fs = 1000;        % Sampling frequency             (Hz)T = 1/Fs;
T  = 1/Fs;        % Sampling period                (sec)
L  = 1500;        % Length of signal               (number)
t  = (0:L-1)*T;   % Time vector, 0 to 1.5 seconds  (sec)

% Generate Harmonic function in noise and plot it
S = A1*sin(2*pi*f1*t) + A2*sin(2*pi*f2*t);   % harmonic function
X = S + A3*randn(size(t));                    % white noise

plot(1000*t(1:50),X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% Take the FFT, convert to abs magnitude, single sided spectrum
Y  = fft(X);                 % complex two-sided FFT
P2 = abs(Y/L);               % normalize magnitude by # of samples
P1 = P2(1:L/2+1);            % get postive half plus DC value
P1(2:end-1) = 2*P1(2:end-1); % double it for single sided but not DC value

% Define the frequency axis and plot the Single Sided Mag. Spectrum
Fmax  = Fs/2;                % Max Freq = 1/2 the Sample Freq
Fstep = Fs/L;                % Freq step size = Fsample/(number of samples)
freq  = Fstep*(0:L/2);       % Make the appropriate frequency vector

figure,plot(freq,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)'), ylabel('|P1(f)|')
close all
