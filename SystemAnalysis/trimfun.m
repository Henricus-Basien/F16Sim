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

global USE_SI_UNITS feet_to_m lbf_to_N

% USE_SI_UNITS = 1;
% feet_to_m    = 0.3048;
% lbf_to_N     = 4.448222;
% lb_ft2_to_Pa = 47.880258888889;

% Implementing limits:
% Thrust limits
tl_l = 1000;
tl_u = 19000;

if (USE_SI_UNITS == 1)
    tl_l = tl_l*lbf_to_N
    tl_u = tl_u*lbf_to_N
end

if UX0(1) > tl_u
    UX0(1) = tl_u;
elseif UX0(1) < tl_l
    UX0(1) = tl_l;
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

    alt_ = altitude;
    vel_ = velocity;

    if (USE_SI_UNITS == 1)
        alt_ = alt_/feet_to_m;
        vel_ = vel_/feet_to_m;
    end

    rho0 = 2.377e-3; tfac = 1 - 0.703e-5*alt_;
    temp = 519*tfac; if (alt_ >= 35000) temp = 390; end;
    rho = rho0*tfac^4.14;
    qbar = 0.5*rho*vel_^2;
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

xu = [  0             ... %npos (ft|m)
        0             ... %epos (ft|m)
        altitude      ... %altitude (ft|m)
        phi*(pi/180)  ... %phi (rad)
        UX0(3)        ... %theta (rad)
        psi*(pi/180)  ... %psi (rad)
        velocity      ... %velocity (ft|m/s)
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

OUT = feval('F16Sim',xu);
% if (USE_SI_UNITS == 1)
%     OUT = feval('F16Sim_SI',xu);
% else
%     OUT = feval('F16Sim_IU',xu);%feval('nlplant',xu);
% end

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