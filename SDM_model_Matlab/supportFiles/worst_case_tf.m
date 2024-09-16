function [T_wc] = worst_case_tf(T)
%WORST_CASE_TF Summary of this function goes here
%   Detailed explanation goes here
[wcg,wcu] = wcgain(T);
T_wc = usubs(T,wcu);
end

