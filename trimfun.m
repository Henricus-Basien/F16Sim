%=====================================================
%      F16 nonlinear model trim cost function
%   for longitudinal motion, steady level flight
% (cost = sum of weighted squared state derivatives)
%
% Author: T. Keviczky
% Date:   April 29, 2002
%
%      Added addtional functionality.
%      This trim function can now trim at three 
%      additional flight conditions
%         -  Steady Turning Flight given turn rate
%         -  Steady Pull-up flight - given pull-up rate
%         -  Steady Roll - given roll rate
%
% Coauthor: Richard S. Russell
% Date:     November 7th, 2002
%
%
%=====================================================
%%**********************************************%%  
%   Altered to work as a trimming function       %
%   for the  HIFI F_16 Model                     %
%%**********************************************%%

function [cost, Xdot, xu] = trimfun(UX0)

global phi psi p q r phi_weight theta_weight psi_weight
global altitude velocity fi_flag_Simulink

% Implementing limits:
% Thrust limits
if UX0(1) > 19000
    UX0(1) = 19000;
elseif UX0(1) < 1000
    UX0(1) = 1000;
end;

% elevator limits
if UX0(2) > 25
    UX0(2) = 25;
elseif UX0(2) < -25
    UX0(2) = -25;
end;

% angle of attack limits
if (fi_flag_Simulink == 0)
  if UX0(3) > 45*pi/180
    UX0(3) = 45*pi/180;
  elseif UX0(3) < -10*pi/180
    UX0(3) = -10*pi/180;
  end
elseif (fi_flag_Simulink == 1)
  if UX0(3) > 90*pi/180
    UX0(3) = 90*pi/180;
  elseif UX0(3) < -20*pi/180
    UX0(3) = -20*pi/180;
  end
end

%  Aileron limits
if UX0(4) > 21.5
    UX0(4) = 21.5;
elseif UX0(4) < -21.5
    UX0(4) = -21.5;
end;

% Rudder limits
if UX0(5) > 30
    UX0(5) = 30;
elseif UX0(5) < -30
    UX0(5) = -30;
end;

if (fi_flag_Simulink == 1)
    % Calculating qbar, ps and steady state leading edge flap deflection:
    % (see pg. 43 NASA report)
    rho0 = 2.377e-3; tfac = 1 - 0.703e-5*altitude;
    temp = 519*tfac; if (altitude >= 35000) temp = 390; end;
    rho = rho0*tfac^4.14;
    qbar = 0.5*rho*velocity^2;
    ps = 1715*rho*temp;
    
    dLEF = 1.38*UX0(3)*180/pi - 9.05*qbar/ps + 1.45;
    
elseif (fi_flag_Simulink == 0)
    dLEF = 0.0;
end

% Verify that the calculated leading edge flap
% have not been violated.
if (dLEF > 25)
    dLEF = 25;
elseif (dLEF < 0)
    dLEF = 0;
end;

xu = [  0             ... %npos (ft)
        0             ... %epos (ft)
        altitude      ... %altitude (ft)
        phi*(pi/180)  ... %phi (rad)
        UX0(3)        ... %theta (rad)
        psi*(pi/180)  ... %psi (rad)
        velocity      ... %velocity (ft/s)
        UX0(3)        ... %alpha (rad)
        0             ... %beta (rad)
        p*(pi/180)    ... %p (rad/s)
        q*(pi/180)    ... %q (rad/s)
        r*(pi/180)    ... %r (rad/s)
        UX0(1)        ... %thrust (lbs)
        UX0(2)        ... %ele (deg)
        UX0(4)        ... %ail (deg)
        UX0(5)        ... %rud (deg)
        dLEF          ... %dLEF (deg)
        fi_flag_Simulink ...% fidelity flag
        ]';

OUT = feval('nlplant',xu);

Xdot = OUT(1:12,1);

% Create weight function
weight = [  0            ...%npos_dot
            0            ...%epos_dot
            5            ...%alt_dot
            phi_weight   ...%phi_dot
            theta_weight ...%theta_dot
            psi_weight   ...%psi_dot
            2            ...%V_dot
            10           ...%alpha_dpt
            10           ...%beta_dot
            10           ...%P_dot
            10           ...%Q_dot
            10           ...%R_dot
            ];

cost = weight*(Xdot.*Xdot);