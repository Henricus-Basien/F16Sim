function out = hinfnorm(sys,ttol,~)
%HINFNORM  Compute H-infinity norm of a linear dynamic system.
%
%   NINF = HINFNORM(SYS) returns the H-infinity norm NINF of the dynamic  
%   system SYS, defined as the peak gain of the frequency response when 
%   SYS is stable, and Inf when SYS is unstable. For MIMO systems, the
%   gain at a particular frequency is measured by the largest singular 
%   value of the frequency response at this frequency.
%
%   NINF = HINFNORM(SYS,TOL) specifies the relative accuracy TOL for the
%   computed value NINF. By default NINF is computed with 1% accuracy
%   (TOL=1e-2).
%
%   [NINF,FPEAK] = HINFNORM(SYS,...) also returns the frequency FPEAK
%   (in rad/TimeUnit) at which the gain achieves its peak value NINF.
%   HINFNORM returns FPEAK=NaN when SYS is unstable.
% 
%   If SYS is an array of dynamic systems, HINFNORM returns an array 
%   of the same size where NINF(k) = HINFNORM(SYS(:,:,k)).
%
%   Note: HINFNORM is the same as getPeakGain for stable systems.
%   
%   See also getPeakGain, freqresp, sigma, DynamicSystem.

% OLD HELP
% function out = hinfnorm(sys,ttol,iiloc)
%
%   Calculates the H_infinity norm of stable, SYSTEM
%   matrices, using the Hamiltonian method, or of VARYING
%   matrices, using PKVNORM. The second argument is used
%   for SYSTEM matrices, and is the relative tolerance
%   between the upper and lower bounds for the infinity
%   norm when the search is terminated. There is an optional
%   third argument, for a initial frequency guess, if desired.
%   for SYSTEM matrices, OUT is a 1x3 row vector, with the
%   lower bound, upper bound, and the frequency where the lower
%   bound occurs. The default value for TTOL is 0.001.
%
%   See also: H2SYN, H2NORM, HINFCHK, HINFSYN, and HINFFI.

%   Copyright 1991-2013 The MathWorks, Inc.
narginchk(1,3);
ni = nargin;

if ni<2 || isempty(ttol)
    ttol = 1e-3;
else
    if ~(isnumeric(ttol) && isscalar(ttol) && isreal(ttol) && ttol>0)
        error(message('Robust:analysis:hinfnorm1'));
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
    sys=ss(a,b,c,d);
    if max(real(eig(a)))< 0 % stable:compute gain
        [f,w] = getPeakGain(sys,ttol);
        out = [f f*(1+ttol) w];
    else                    % unstable:return inf
        warning(message('Robust:analysis:hinfnorm2'));
        out = [inf inf 0];
    end
end
