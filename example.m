%% init
clear
close all

%% MSD specification
% number of mass points
N = 3;
% mass of each point in kg
M = [1; 100; 1];
% stiffness of springs in N/m
K = [1; 1; 1];
% damping coeficient in Ns/m
C = [1; 1; 1];

% get continuous time model
[a, b, c, d] = msd(N, 'M', M, 'K', K, 'C', C);

% get continuous time model
% sampling time
Ts = 0.5;
[ad, bd, cd, dd] = msd(N, 'M', M, 'K', K, 'C', C, 'Ts', Ts);

% get both models
[sysc, sysd] = msd(N, 'both', true, 'Ts', Ts);