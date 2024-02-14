%%% flyback converter calculator
% Author: Habonroof <johnson35762@gmail.com>
% for more detail, please visit www.Habonroofplayground.com

close all;
clear;
clc;
%% parameter setup
Vin_min = 5;
Vin_max = 20;     % input voltage
eff = 0.80;       % efficiency
D_max = 0.45;      % maximun duty
Fs = 500*10^3;    % switching frequency
Po = 6;         % output power
Kfr = 1;          % ripple factor, for DCM, Kfr = 1, for BCM,CCM Kfr = 0.4-0.8.
Pin = Po / eff;
%% transformer primary inductance, depending on Vin
Lp_max = (D_max^2 * Vin_max^2) / (2 * Fs * Pin * Kfr) * 10^6;
fprintf("Vin = %d \tLp = %.2f uH \n\r", Vin_max, Lp_max);

Lp_min = (D_max^2 * Vin_min^2) / (2 * Fs * Pin * Kfr) * 10^6;
fprintf("Vin = %d \tLp = %.2f uH \n\r", Vin_min, Lp_min);

Lp_arr = [Lp_max, Lp_min];
Lp = max(Lp_arr);
fprintf("Choose Lp = %.2fuH\n\r", Lp);

