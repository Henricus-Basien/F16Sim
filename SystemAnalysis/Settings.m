%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Default Conditions
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

global altitude0
global velocity0
altitude0 = 15000;
velocity0 = 500;

global FlightCondition
FlightCondition = 1;

global AcceptFirstIteration
AcceptFirstIteration = 1;

%----------------------------------------
% Plotting
%----------------------------------------

global PlotPoles
global PlotBode
PlotPoles = "n";%"y";
PlotBode  = "n";%"y";

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% SI Functionality
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

global USE_SI_UNITS
global feet_to_m   
global lbf_to_N    
global lb_ft2_to_Pa
global lu
global fu

USE_SI_UNITS = 0;%1;
feet_to_m    = 0.3048;
lbf_to_N     = 4.448222;
lb_ft2_to_Pa = 47.880258888889;

if (USE_SI_UNITS == 1)
    lu = "m";
    fu = "N";
else
    lu = "ft";
    fu = "lbf";
end


% disp("Settings are set!")