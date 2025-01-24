function [Fd, Qd, Bd] = continuousToDiscrete(Fc, Qc, Bc, dT)

% Use Van Loan's algorithm to work out a discrete => continuous
% transformation. If Bc is passed in and Bd is requested, the control input
% matrix will be converted as well

% This kludgy logic is used because Bd is optional
if (nargin == 3)
    dT = Bc;
end

n = size(Fc, 1);

bigA = zeros(2*n);

bigA(1:n,1:n) = -Fc * dT;
bigA(1:n, n+1:end) = Qc * dT;
bigA(n+1:end, n+1:end) = Fc' * dT;

bigB=expm(bigA);

Fd = bigB(n+1:end, n+1:end)';
Qd = Fd * bigB(1:n, n+1:end);

% Compute Bd if requested
Bd = [];

if (nargin == 4)
    nu = size(Bc, 2);
    bigA = zeros(n + nu);
    bigA(1:n,1:n) = Fc * dT;
    bigA(1:n, n+1:end) = Bc * dT;
    bigB=expm(bigA);
    Bd = bigB(1:n, n+1:end);
end