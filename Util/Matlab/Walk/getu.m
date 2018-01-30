function [ u ] = getu( f, p, N, k )
%GETU Summary of this function goes here
%   Detailed explanation goes here
u = 0;
for j=1:N,
    u = u + Gd(j) * p(k+j);
end