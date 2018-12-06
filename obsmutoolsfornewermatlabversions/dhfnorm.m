function out = dhfnorm(sys,ttol,h,~)
% function out = dhfnorm(sys,ttol,h,iiloc)
%
%   Calculates the H_infinity norm of stable, discrete time SYSTEM
%   matrix, SYS, via a bilinear transformation and HINFNORM, or of VARYING
%   matrices, using PKVNORM. The second argument, TTOL, is used
%   for SYSTEM matrices, and is the relative tolerance
%   between the upper and lower bounds for the infinity
%   norm when the search is terminated. TTOL is optional with a
%   default value of 0.001. The optional third augment, H,
%   is the sample period (default = 1). The optional fourth
%   argument, IILOC, is for an initial frequency guess for
%   the worst case frequency, if desired.
%
%   For SYSTEM matrices, OUT is a 1x3 row vector, with the
%   lower bound, upper bound, and the frequency where the lower
%   bound occurs. For CONSTANT matrices, OUT is a 1x3 row vector,
%   with the NORM of the matrix in the 1st two entries and 0 for
%   the frequency. OUT is the PKVNORM of SYS if SYS is a VARYING
%   matrix.
%
%   See also: DHFSYN, HINFNORM, H2SYN, H2NORM, HINFCHK, HINFSYN,
%              HINFFI, NORM, PKVNORM, SDHFNORM, and SDHFSYN.

%   Copyright 1991-2011 MUSYN Inc. and The MathWorks, Inc.

narginchk(1,4);
ni = nargin;

if ni<2 || isempty(ttol) % tol set & check
    ttol=1e-3;
else
    if ~(isnumeric(ttol) && isscalar(ttol) && isreal(ttol) && ttol>0)
        error(message('Robust:analysis:dhfnorm1'));
    end
end

if ni<3 || isempty(h) % h set & check
    h=1;
else
    if ~(isnumeric(h) && isscalar(h) && isreal(h) && h>0)
        error(message('Robust:analysis:dhfnorm2'));
    end
end

[systype,~,~,~] = minfo(sys);

if strcmp(systype,'vary') % varying matrices
    out = pkvnorm(sys);
elseif strcmp(systype,'cons') % constant matrix
    out = norm(sys);
    out = [out out 0];
else % SYSTEM matrices
    [a,~,~,~] = unpck(sys);
    if max(abs(eig(a)))>= 1 % error if unstable
        error(message('Robust:analysis:dhfnorm3'));
    end     
    sys_c=bilinz2s(sys,h); % transform to cont sys by bilinear trans
    [a,b,c,d] = unpck(sys_c);
    sys_c=ss(a,b,c,d);
    [f,w] = getPeakGain(sys_c,ttol); % compute norm
    wd=(2/h)*atan(w*h/2); % discrete frequency
    out = [f f*(1+ttol) wd];
end