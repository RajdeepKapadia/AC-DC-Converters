clc;
close all;
clear all; 

%% Given 

% Plant 

L = 0.2e-3; 
C = 6e-3; 
R_non = 300; 
wo = 2*pi*60; 
Po = 22500;
Vo = 1500; 
eff = 0.98; 
Vrms = 277.12; 

% PI 

kp = 3e-3;
ti = 1e-3;  

%% Defining frequency vector in Hertz up to 50 kHz 
freq_Hz = logspace(0, log10(50000), 1000); % Frequency in Hertz

%% Convert frequency from Hertz to rad/s
freq_rad_s = 2 * pi * freq_Hz; % Conversion to radians per second

%% Irms calculation
Irms = Po/(eff*Vrms*3);  % 3 for each phase. 

%% Zeros & poles calculation for R_non. 

% Zero 

wz1 = 2/(R_non*C); 

% Poles Calculation 

wp1 = 1/(R_non*C); 

wp2_1 = (6*(Vrms)^2)/(L*C*(Vo)^2); 
wp2_2 = (6*L*(wo)^2*(Irms)^2)/(C*(Vo)^2); 

wp2 = (wo)^2 + wp2_1 + wp2_2;  

wp3 = (wo^2)/(R_non*C);

% Transfer function of il/d for 3-phase currrent loop

num1 = [0 -Vo -Vo*wz1 0]; 
den1 = [2*L 2*L*wp1 2*L*(wp2) 2*L*(wp3)];

% Gp = tf(num1,den1); 
% bode(Gp,freq_rad_s);

%% Transfer function of PI controller 

num = [kp*ti kp];
den = [ti 0]; 

%% Compensated Network 

% Close loop transfer function 

Gp = tf(num1, den1);
Gc = tf(num, den); 

G = Gc*Gp; 
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