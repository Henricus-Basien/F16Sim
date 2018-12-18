%****************************************************************************************************
% Settings
%****************************************************************************************************

%if exist("SettingSet") == 0 || isempty(SettingSet)

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

    RunQ5 = 1
    RunQ6 = 1
    RunQ7 = 1
    RunQ8 = 1

    PlotQ5 = 1
    PlotQ6 = 1
    PlotQ7 = 1
    PlotQ8 = 1

    figpath ='OutputFigures';
    if ~exist(figpath, 'dir')
        mkdir(figpath)
    end
    figext  = '.png';
    dpi = '-r300';

    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Tolerances
    %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    ApplyStateSpaceSimplification = 1;

    e         = 0.0001; %Tolerance used by SimplifyStatespace
    e_minreal = 0.001 ; %Tolerance of Minreal commands for pole-zero cancellations

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
