clc;
clear all;
close all;

%% Given 

% Plant Parameters 
L = 10e-3; 
C = 4000e-6; 
d = 0.567; 
R_nom = 75;
Vout = 1500;
V_rms = 649; 

% PI Parameters 
kp = 1.2; 
ti = 0.5; 

%% Defining frequency vector in Hertz up to 50 kHz 
freq_Hz = logspace(0, log10(50000), 1000); % Frequency in Hertz

%% Convert frequency from Hertz to rad/s
freq_rad_s = 2 * pi * freq_Hz; % Conversion to radians per second

%% Transfer function - Plant 

num1 = [0 R_nom*V_rms]; 
den1 = [C*R_nom*Vout Vout]; 

Gp = tf(num1,den1); 

%% Transfer function of PI controller 

num = [kp*ti kp];
den = [ti 0]; 
Gc = tf(num, den); 

%% Compensated System
G = Gp * Gc; 

%% Bode Plot

% Retrieve Bode plot data
[mag, phase, wout] = bode(G, freq_rad_s);

% Convert frequencies from rad/s to Hz
freq_Hz_from_bode = wout / (2 * pi);

% Plot magnitude and phase on separate plots
figure;
subplot(2,1,1);
semilogx(freq_Hz_from_bode, 20*log10(squeeze(mag)));
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Gain Plot')
grid on;

subplot(2,1,2);
semilogx(freq_Hz_from_bode, squeeze(phase));
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title ('Phase Plot')
grid on;
