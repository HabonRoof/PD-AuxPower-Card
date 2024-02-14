%%% flyback converter calculator
% Author: Habonroof <johnson35762@gmail.com>
% for more detail, please visit www.Habonroofplayground.com

close all;
clear;
clc;
%% parameter setup
Vin_min = 5;      % input voltage
Vin_max = 20;
Vo_max = 3.3;     % output voltage
Vo_min = 5;
Vd = 0.4;         % diode conduct voltage
D_max = 0.45;     % maximun duty

%%% calculate turns ratio of transformer
% Ns1 = Np/Ns choose the smallest one for worst case
Ns1 = (Vin_max * D_max) / ((1 - D_max) * (Vo_max + Vd));
fprintf("turns ratio when Vin = %d\tVo = %d  \tNs1=%.2f\n\r",Vin_max, Vo_max, Ns1);

Ns2 = (Vin_max * D_max) / ((1 - D_max) * (Vo_min + Vd));
fprintf("turns ratio when Vin = %d\tVo = %d \tNs1=%.2f\n\r",Vin_max, Vo_min, Ns2);

Ns3 = (Vin_min * D_max) / ((1 - D_max) * (Vo_max + Vd));
fprintf("turns ratio when Vin = %d\tVo = %d \tNs1=%.2f\n\r",Vin_min, Vo_max, Ns3);

Ns4 = (Vin_min * D_max) / ((1 - D_max) * (Vo_min + Vd));
fprintf("turns ratio when Vin = %d\tVo = %d \tNs1=%.2f\n\r",Vin_min, Vo_min, Ns4);
N=[Ns1, Ns2, Ns3, Ns4];
Ns = min(N);

fprintf("Choose worest case, Np:Ns = %.2f\n\r",Ns);