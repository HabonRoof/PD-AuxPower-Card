%%% flyback converter calculator
% Author: Habonroof <johnson35762@gmail.com>
% for more detail, please visit www.Habonroofplayground.com

close all;
clear;
clc;
%% parameter setup
Vin_min = 5;
Vin_max = 20;     % input voltage
Vout_max = 24;
Vout_min = 12;
eff = 0.80;       % efficiency
D_max = 0.45;     % maximun duty
D_min = 0.45;
Fs = 500*10^3;    % switching frequency
Po = 6;         % output power

Ae = 8.41;  % transformer core cross-section area (mm2)
ns = 0.17;       % turns ratio
B_max = 0.3;     % maximum magnetatic flux (Tesla)

%% Lp min
fprintf("For Vin = %d, Vo = %d\n\r", Vin_min, Vout_max);
Lp = 10.8*10^-6; % primary inductance
Ip_peak = 3.45; % peak primary current

Np = (Lp * Ip_peak * 10^6) / (B_max * Ae);
Ns = Np / ns;
fprintf("Np = %d Ns = %d\n\n\r",round(Np), round(Ns));

Np2 = (Vin_max * D_min) / (Fs * B_max * 10^4 * Ae * 10^-6) * 10^4;
Ns2 = Np2 / ns;
fprintf("Np2 = %d Ns2 = %d\n\n\r",round(Np2), round(Ns2));
