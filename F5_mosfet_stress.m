%%% flyback converter calculator
% Author: Habonroof <johnson35762@gmail.com>
% for more detail, please visit www.Habonroofplayground.com

close all;
clear;
clc;

Vin_min = 5;
Vin_max = 20;     % input voltage
eff = 0.80;       % efficiency
D_max = 0.45;      % maximun duty
Fs = 500*10^3;    % switching frequency
Po = 6;         % output power
Kfr = 1;          % ripple factor

% calculate maximum voltage stress on MOSFET, choose voltage rating bigger
% then 1.2 times of this value
Vds_max = Vin_max + (D_max * Vin_min)/(1-D_max);
fprintf("Vds max = %.2f V \n\r", Vds_max);
fprintf("Choose %.2fV MOSFET \n\r", 1.2 * Vds_max);
