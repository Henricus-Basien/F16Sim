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
StateAliases

%================================================================================
% Excercises
%================================================================================

%input('Press ENTER to run Full Analysis...');
if RunQ5
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
    fprintf('                             Q5                             \n')
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.2                  \n')
    fprintf('----------------------------------------\n')

    fprintf('Trimming Linearized Model\n')
    FindF16Dynamics

    if ApplyStateSpaceSimplification == 1
        SimplifyStatespace
    end

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.3                  \n')
    fprintf('----------------------------------------\n')

    %C_lo
    %D_lo

    B_Ue = B_lo(:,Ue);
    C_Nz = C_lo(YNz,:);
    D_Ue = D_lo(:,Ue);

    C_Nz
    D_Ue

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.4                  \n')
    fprintf('----------------------------------------\n')

    Nz_index = find(C_Nz,NrOutputs)
    PrintStateNames(NZindex,'Elements Nz depends on: ')

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.5                  \n')
    fprintf('----------------------------------------\n')

    tf_Ue_Nz = minreal(tf(C_lo(YNz,:) * (inv((s*eye(NrStates)-A_lo))*B_lo(:,Ue))),e_minreal)

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.6                  \n')
    fprintf('----------------------------------------\n')

    fprintf('Negative Elevator step input output Nz\n');

    figure(1);
    PlotElevatorStepInput(tf_Ue_Nz)

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.7                  \n')
    fprintf('----------------------------------------\n')

    tf_Ue_Nz_zeros = zero(tf_Ue_Nz)
    tf_Ue_Nz_poles = pole(tf_Ue_Nz)

    figure(2);
    grid on
    pzmap(tf_Ue_Nz)
    title('Ue->YNz Pole-Zero Map');

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.8                  \n')
    fprintf('----------------------------------------\n')

    fprintf('When performing a pull-up maneuver the aircraft pitch moment is created due to the negative lift on the horizontal tail surface; this negative lift initially pushes the aircraft downwards slightly before the increase lift of the main wing due to the increase in AoA can take effect. \n');

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.9                  \n')
    fprintf('----------------------------------------\n')

    xa_ = [0,5,5.9,6,7,15];

    figure(3);
    for xa = xa_
        fprintf('Analyzing xa_ %f\n: ',xa)
        FindF16Dynamics
        if ApplyStateSpaceSimplification == 1
            SimplifyStatespace
        end
        tf_Ue_Nz_ = minreal(tf(C_lo(YNz,:) * (inv((s*eye(NrStates)-A_lo))*B_lo(:,Ue))),e_minreal);

        PlotElevatorStepInput(tf_Ue_Nz_)
    end
end

fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
fprintf('                             Q6                             \n')
fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

if RunQ6

	fprintf('----------------------------------------\n')
    fprintf('                  Q6.1                  \n')
    fprintf('----------------------------------------\n')

    FindF16Dynamics
    if 0%1
        %A_lo
        %B_lo
        %C_lo
        %D_lo

        tf_sys = minreal(tf(C_lo * (inv((s*eye(NrStates)-A_lo))*B_lo)),e_minreal)
        pzmap(tf_sys)
    end
    
    %..............................
    % Get Simplified Matrics
    %..............................
   
    %A_lo
    
    if 0%1
        %--- Longitudinal ---
        A_long = A_lo(longitudinal_states,longitudinal_states);
        longitudinal_states
        PrintStateNames(longitudinal_states,"longitudinal_states: ")
        A_long

        %--- Lateral---
        A_lat = A_lo(lateral_states,lateral_states);
        lateral_states
        PrintStateNames(lateral_states,"lateral_states: ")
    else
        A_long = A_longitude_lo
        A_lat  = A_lateral_lo
        B_long = B_longitude_lo
        B_lat  = B_lateral_lo
        C_long = C_longitude_lo
        C_lat  = C_lateral_lo
        D_long = D_longitude_lo
        D_lat  = D_lateral_lo
    end
    
    %..............................
    % Get Transfer Functions
    %..............................
    
    tf_long = minreal(tf(C_long * (inv((s*eye(size(A_long,1))-A_long))*B_long)),e_minreal)
    tf_lat  = minreal(tf(C_lat  * (inv((s*eye(size(A_lat ,1))-A_lat ))*B_lat )),e_minreal)

    %..............................
    % Get Poles & Zeros
    %..............................
    
    tf_long_zeros = zero(tf_long)
    tf_long_poles = pole(tf_long)
    tf_lat_zeros  = zero(tf_lat )
    tf_lat_poles  = pole(tf_lat )

    %..............................
    % Plot Pole-Zero maps
    %..............................

    figure(4);
    grid on
    pzmap(tf_long)
    title('Longitudinal Pole-Zero Map')

    figure(5);
    grid on
    pzmap(tf_lat)
    title('Lateral Pole-Zero Map')

end

%================================================================================
% Addendum
%================================================================================

input('Press ENTER to close the Analysis...');
close all

%****************************************************************************************************
% Functions
%****************************************************************************************************

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Print State Names
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function PrintStateNames(index,prefix)
   StateAliases
   fprintf(prefix)
    for i = index
        fprintf('%s, ',StateNames(i,:));
    end
    fprintf('\n');
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Plot Elevator Step Input
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function PlotElevatorStepInput(tf_)
	opt = stepDataOptions('StepAmplitude', -1);
	T = 0:0.01:6;   
	grid on
	step(tf_, T, opt);
	title('Negative Elevator step input');
	xlabel('Time [s]');
	ylabel('Normal acceleration in z [g]');
end