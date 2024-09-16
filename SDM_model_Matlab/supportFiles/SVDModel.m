function [A,B,C,D] = SVDModel(m,k,c)
%SVDMODEL Summary of this function goes here
%   Detailed explanation goes here
A = [-c/m, -k/m;
    1, 0];
B = [1/m;
    0];
C = [0, 1];
D = [0];
end

