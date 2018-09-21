function varargout = msd(varargin)
%% State space generator for Mass-Spring-Damper system of N carts.  
%   
%   [A, B, C, D] = MSD(N) generates state space model in continuous time 
%   representing N mass points connected with spring and damper with each
%   mass M = 10kg, stiffness K = 20N/m, and damping coeeficient C = 10Ns/m
%   
%   states X:
%   first N elements of the state vector are positions of carts, rest of
%   the state vector are speeds of carts
%
%   input U:
%   the input force affects only the first cart, the last mass point is
%   connected with spring and damper to a wall
%
%   output Y:
%   the output vector contains positions of each cart
%
% Possible optional arguments:
%   M [kg] mass of N carts (M is vector)
%       [A, B, C, D] = MSD(N, 'M', M) 
%   K [N/m] stiffness of N carts (K is vector)
%       [A, B, C, D] = MSD(N, 'K', K)
%   C [Ns/m] damping coefficent of N carts (C is vector)
%       [A, B, C, D] = MSD(N, 'C', C)
%
%   For discrete time system specify Ts
%       [A, B, C, D] = MSD(N, 'Ts', Ts)
%
%   Ctime and Dtime output
%       [sysc, sysd] = MSD(N, 'both', true)
%   
%  Values M, K, and C can be provided as scalars or vectors of size N. If 
%  a scalar value is chosen, then all carts will have equal physical 
%  properties.
%

%% source code
% defaults
default.M  = 10;
default.C  = 10;
default.K  = 20;
default.Ts = 0;

% input parsing
p = inputParser;
p.addRequired('N', @(x) validateattributes(x,{'numeric'},{'integer','positive'}));
p.addParameter('M', default.M, @(x) validateattributes(x,{'numeric'},{'positive'}));
p.addParameter('C', default.C, @(x) validateattributes(x,{'numeric'},{'positive'}));
p.addParameter('K', default.K, @(x) validateattributes(x,{'numeric'},{'positive'}));
p.addParameter('Ts', default.Ts, @(x) validateattributes(x,{'numeric'},{'positive'}));
p.addParameter('both', false, @(x) validateattributes(x,{'logical'},{'scalar'}));

parse(p, varargin{:});
opt = p.Results;

if opt.both == false && opt.Ts  ==  0
    fprintf('\tProviding a continuous time model. Specify Ts for discrete time.\n');
end

if opt.both == true && opt.Ts == 0
    opt.Ts = 0.1;
    warning('Sampling time Ts not specified, using Ts = 0.1s')
end

if length(opt.M)  ==  1
    opt.M = ones(opt.N, 1)*opt.M;
elseif length(opt.M) ~= opt.N
    error(['Number of masses does not match number of carts. ', ...
    'Choose M as a vector of N masses or a scalar for all masses to be equal.'])
end

if length(opt.K)  ==  1
    opt.K = ones(opt.N, 1)*opt.K;
elseif length(opt.K) ~= opt.N
    error(['Number of stiffness constants does not match number of carts.', ...
    'Choose K as a vector of N stiffness constants or a scalar for all stiffness constants to be equal.'])
end

if length(opt.C)  ==  1
    opt.C = ones(opt.N, 1)*opt.C;
elseif length(opt.C) ~= opt.N
    error(['Number of damping coefficents does not match number of carts.', ...
    'Choose K as a vector of N damping coefficents or a scalar for all damping coefficents to be equal.'])
end

M = opt.M;
K = opt.K;
C = opt.C;

%
if opt.N > 1
    % initialize state space matrices
    B= zeros(opt.N*2,1);
    AK= zeros(opt.N);
    AC= zeros(opt.N);
    
    %% calculating state space matrix A
    j = 1;
    for row = 1:opt.N
        for col = 1:opt.N
            m = M(j);
            if row == 1
                if col == j
                    AK(row,col) = -K(j)/m;
                    AC(row,col) = -C(j)/m;
                elseif col == j+1
                    AK(row, col) = K(j)/m;
                    AC(row, col) = C(j)/m;
                end
            elseif row == opt.N
                if col == j
                    AK(row, col) = -(K(j)+K(j-1))/m;
                    AC(row, col) = -(C(j)+C(j-1))/m;
                elseif col == j-1
                    AK(row, col) = K(j-1)/m;
                    AC(row, col) = C(j-1)/m;
                end
            else
                if col == j
                    AK(row, col) = -(K(j) + K(j - 1))/m;
                    AC(row, col) = -(C(j) + C(j - 1))/m;
                elseif col ==  j - 1
                    AK(row, col) = K(j - 1)/m;
                    AC(row, col) = C(j - 1)/m;
                elseif col ==  j + 1
                    AK(row, col) = K(j)/m;
                    AC(row, col) = C(j)/m;
                end
            end
        end
        j = j + 1;
    end
    A= [zeros(opt.N) eye(opt.N); AK AC] ;
else
    A=[0 1;-K/M -C/M];
end

%% calculating state space matrix B
B(opt.N+1,1)= 1/M(1) ;

C = eye(opt.N, opt.N*2);
D = zeros(size(C, 1), size(B, 2));

if opt.Ts  ~=  0
    sysc = ss(A, B, C, D);
    sysd = c2d(sysc, opt.Ts);
    A = sysd.a;
    B = sysd.b;
    C = sysd.c;
    D = sysd.d;
end

if opt.both == false
    varargout{1} = A;
    varargout{2} = B;
    varargout{3} = C;
    varargout{4} = D;
else
    if nargout > 2
        error('Too many output arguments. Try to call [sysd, sysc] = msd(N, ''both'', true)')
    end
    varargout{1} = sysc;
    varargout{2} = sysd;
end




end