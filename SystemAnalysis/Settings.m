%****************************************************************************************************
% Settings
%****************************************************************************************************

%if exist("SettingSet") == 0 || isempty(SettingSet)

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Default Conditions
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    global altitude00
    global velocity00
    altitude00 = 10000; % [ft]
    velocity00 = 900  ; % [ft/s]  

    global FlightCondition
    FlightCondition = 1;

    global NrIteration
    NrIteration = 3;%1;

    %----------------------------------------
    % Printing
    %----------------------------------------

    global PrintDynamicsResults
    PrintDynamicsResults = "n";%"y";

    %----------------------------------------
    % Plotting
    %----------------------------------------

    global PlotPoles
    global PlotBode
    PlotPoles = "n";%"y";
    PlotBode  = "n";%"y";

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Exercises
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    RunQ5 = true;
    RunQ6 = true;
    RunQ7 = true;
    RunQ8 = true;

    PlotQ5 = true;
    PlotQ6 = true;
    PlotQ7 = true;
    PlotQ8 = true;

    ShowDominantPoles = false; % true;

    figpath ='OutputFigures';
    if ~exist(figpath, 'dir')
        mkdir(figpath)
    end
    figext  = '.png';
    dpi = '-r300';

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Tolerances
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    ApplyStateSpaceSimplification = 0;%1;

    e_ss      = 0.0001;     %Tolerance used by SimplifyStatespace
    e_minreal = 0.002 ;     %Tolerance of Minreal commands for pole-zero cancellations
    e         = e_minreal;  %Tolerance used for Duplicate Pole Removal

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

    SettingSet = 1;

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Limits
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    thrust_max = 19000; % [lbf]
    thrust_min = 1000 ; % [lbf]

    if USE_SI_UNITS
        thrust_max = thrust_max*lbf_to_N;
        thrust_min = thrust_min*lbf_to_N;
    end

    elevator_max =  25; % [deg]
    elevator_min = -25; % [deg]

    disp("Settings are set!")

%end

global xa
xa = 0;

global g0
g0 = 9.80665;
% disp("Settings are set!")
