% function out = linfnorm(sys,ttol,iiloc)
%
%   Calculates the L_infinity norm of stable or unstable SYSTEM
%   matrices, using the Hamiltonian method, or of VARYING
%   matrices, using PKVNORM. The second argument is used
%   for SYSTEM matrices, and is the relative tolerance
%   between the upper and lower bounds for the infinity
%   norm when the search is terminated. There is an optional
%   third argument, for a initial frequency guess, if desired.
%   for SYSTEM matrices, OUT is a 1x3 row vector, with the
%   lower bound, upper bound, and the frequency where the lower
%   bound occurs. The default value for TTOL is 0.001.
%   Identical to HINFNORM, except no stability check is made.
%
%   See also: H2SYN, H2NORM, HINFCHK, HINFNORM, HINFSYN, and HINFFI.

%   Copyright 1991-2011 The MathWorks, Inc.

function out = linfnorm(sys,ttol,~)

narginchk(1,3);
ni = nargin;

if ni<2 || isempty(ttol)
    ttol = 1e-3;
else
    if ~(isnumeric(ttol) && isscalar(ttol) && isreal(ttol) && ttol>0)
        error(message('Robust:analysis:linfnorm1'));
    end
end

[systype,~,~,~] = minfo(sys);

if strcmp(systype,'vary') % varying matrices
    out = pkvnorm(sys);
elseif strcmp(systype,'cons') % constant matrix
    out = norm(sys);
    out = [out out 0];
else % SYSTEM matrices
    [a,b,c,d] = unpck(sys);
    sys = ss(a,b,c,d);
    [f,w] = getPeakGain(sys,ttol);
    out = [f f*(1+ttol) w];
end