%{
Created on Tuesday 11.12.2018
Copyright (??) Henricus N. Basien
Author: Henricus N. Basien
Email: Henricus@Basien.de
%}

%****************************************************************************************************
% AE4301P
%****************************************************************************************************

%================================================================================
% Settings
%================================================================================

Settings

%================================================================================
% Linearize Model
%================================================================================

FindF16Dynamics

%================================================================================
% Initialization
%================================================================================

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Preliminaries
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

s = tf('s');
e_minreal = 0.001; %Tolerance of Minreal commands for pole-zero cancellations

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Aliases
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

%================================================================================
% Excercises
%================================================================================

%a = input('Press ENTER to run Full Analysis...');

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Remove almost zeros!
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

e = 0.0001;
if 1
    for i = 1:NrStates
    	% --- A ---
        for j = 1:NrStates
            if abs(A_lo(i,j))<e
                A_lo(i,j) = 0;
            end
        end
        % --- B ---
        for j = 1:NrInputs
            if abs(B_lo(i,j))<e
                B_lo(i,j) = 0;
            end
        end
        % --- C ---
        for j = 1:NrOutputs
        	if abs(C_lo(j,i))<e
                C_lo(j,i) = 0;
            end
        end
        % --- D ---
        for j = 1:NrInputs
            if abs(D_lo(i,j))<e
                D_lo(i,j) = 0;
            end
        end
    end
end
fprintf('--- 5.3 ---\n')

%C_lo
%D_lo

B_Ue = B_lo(:,Ue);
C_Nz = C_lo(YNz,:);
D_Ue = D_lo(:,Ue);

C_Nz
D_Ue

fprintf('--- 5.4 ---\n')
Nz_index = find(C_Nz,NrOutputs)
fprintf('Elements Nz depends on: ')
for i = Nz_index
	fprintf('%s, ',StateNames(i,:));
end
fprintf('\n');

fprintf('--- 5.5 ---\n')
tf_Ue_Nz = minreal(tf(C_Nz * (inv((s*eye(18)-A_lo))*B_Ue)),e_minreal)

fprintf('--- 5.6 ---\n')
opt = stepDataOptions('StepAmplitude', -1);
T = 0:0.01:6;
[y,t] = step(tf_Ue_Nz, T, opt);
figure(1);
plot(t,y)
xlabel('Time [s]');
ylabel('Normal acceleration in z [g]');


