% parameters.m  --- for SV_pwm test-bench
%   Run this script prior to running svpwm.slx
% Brian Tremaine 12/04/2022
%
Ts= 50E-6;  % pwm period
Fhz= 30.0;  % 
Vbus = 5.0; % vols
tau = 1/(2*pi*1000.0); % pwm filter