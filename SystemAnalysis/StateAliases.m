%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Aliases
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

s = tf('s');

%----------------------------------------
% Input
%----------------------------------------

NrInputs = 4;

Ut = 1;
Ue = 2;
Ua = 3;
Ur = 4;

%----------------------------------------
% State
%----------------------------------------

NrStates = 18;

Xnpos  = 1 ;
Xepos  = 2 ;
Xh     = 3 ;
Xphi   = 4 ;
Xtheta = 5 ;
Xpsi   = 6 ;
Xv     = 7 ;
Xalpha = 8 ;
Xbeta  = 9 ;
Xp     = 10;
Xq     = 11;
Xr     = 12;

longitudinal_states = [7,8,5,11];
lateral_states      = [9,4,10,12];

%----------------------------------------
% Output
%----------------------------------------

NrOutputs = 19;

Ynpos  = Xnpos  ;
Yepos  = Xepos  ;
Yh     = Xh     ;
Yphi   = Xphi   ;
Ytheta = Xtheta ;
Ypsi   = Xpsi   ;
Yv     = Xv     ;
Yalpha = Xalpha ;
Ybeta  = Xbeta  ;
Yp     = Xp     ;
Yq     = Xq     ;
Yr     = Xr     ;
Yanx   = 13     ;
Yany   = 14     ;
Yanz   = 15     ;
YM     = 16     ;
Yqp    = 17     ;
Ysp    = 18     ;

YNz    = 19     ;

%----------------------------------------
% Names
%----------------------------------------

StateNames = [
	'npos ', 
	'epos ', 
	'h    ', 
	'phi  ', 
	'theta', 
	'psi  ', 
	'v    ', 
	'alpha', 
	'beta ', 
	'p    ', 
	'q    ', 
	'r    ', 
	'anx  ', 
	'any  ', 
	'anz  ', 
	'M    ', 
	'qp   ', 
	'sp   '
];