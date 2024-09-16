function [DampRatio, PM, Mt] = OverShoot(OS)
%OVERSHOOT Summary of this function goes here
%   Detailed explanation goes here
A = log(OS/100);
DampRatio = sqrt(A^2/(pi^2+A^2));
PM = DampRatio*100;
Mt = 180/PM/pi;
end

