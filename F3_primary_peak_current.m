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
D_max = 0.45;     % maximun duty
Fs = 500*10^3;    % switching frequency
Po = 6;            % output power
Rsense = 0.1;       % current sensing resistor

%% claculate primary side peak current
%% run 'primary_inductance.m' to calculate the value
Lp = 10.8 * 10^-6;
Pin = Po / eff;

Ip_peak = Pin / (D_max * Vin_min) + (D_max * Vin_min) / (2 * Fs * Lp);
P_rses = Ip_peak^2 * Rsense;

fprintf("primary peak current Ip_peak = %.2fA for Vin = %dV\t\n\r",Ip_peak, Vin_min);
fprintf("Rsense power dissapation = %.2f W\n\r",P_rses);
