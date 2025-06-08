clc;
clear all;
close all;

%% Given 

% Plant Parameters 
L = 1e-3; 
Co = 3e-3; 
Vo = 1000; 
Po = 10e3; 
d = 0.322; 
Vin = 638; 
esr = 0; 
R_non = 66.67; 

% PI Parameters 
kp = 0.105;
ti = 1.5; 
 
%% Defining frequency vector in Hertz up to 50 kHz 
freq_Hz = logspace(0, log10(50000), 1000); % Frequency in Hertz

%% Convert frequency from Hertz to rad/s
freq_rad_s = 2 * pi * freq_Hz; % Conversion to radians per second

%% Transfer function - Plant 

s = tf('s'); 

% Numerator 
N = R_non;  
 
% Denominator 
D =  1 + s*R_non*Co;  

Gp = (N/D); 

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


