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
    PrintStateNames(Nz_index,'Elements Nz depends on: ')

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.5                  \n')
    fprintf('----------------------------------------\n')

    tf_Ue_Nz = minreal(tf(C_lo(YNz,:) * (inv((s*eye(NrStates)-A_lo))*B_lo(:,Ue))),e_minreal)

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.6                  \n')
    fprintf('----------------------------------------\n')

    fprintf('Negative Elevator step input output Nz\n');

    figure(51);
    PlotElevatorStepInput(tf_Ue_Nz, '')
    grid on
    
    fprintf('----------------------------------------\n')
    fprintf('                  Q5.7                  \n')
    fprintf('----------------------------------------\n')

    tf_Ue_Nz_zeros = zero(tf_Ue_Nz)
    tf_Ue_Nz_poles = pole(tf_Ue_Nz)

    if 0
    	figure(52);
    	grid on
    	pzmap(tf_Ue_Nz)
    	t = title('Ue-YNz Pole-Zero Map');
    	print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)
    end

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.8                  \n')
    fprintf('----------------------------------------\n')

    fprintf('When performing a pull-up maneuver the aircraft pitch moment is created due to the negative lift on the horizontal tail surface; this negative lift initially pushes the aircraft downwards slightly before the increase lift of the main wing due to the increase in AoA can take effect. \n');

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.9                  \n')
    fprintf('----------------------------------------\n')

    xa_ = [0,5,5.9,6,7,15];

    figure(53);  
    for xa = xa_
        
        fprintf('Analyzing xa: %f %s\n',xa,lu)
        if 0%1
            FindF16Dynamics
        else
            [A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);
            C_lo
        end
        if ApplyStateSpaceSimplification == 1
            SimplifyStatespace
        end
        tf_Ue_Nz_ = minreal(tf(C_lo(YNz,:) * (inv((s*eye(NrStates)-A_lo))*B_lo(:,Ue))),e_minreal);
        name = 'xa = ' + string(xa);
        PlotElevatorStepInput(tf_Ue_Nz_, name)
        hold on
    end
    legend('Location','southeast')
    t = title('xa Shift');
    print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)
    hold off

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.10                 \n')
    fprintf('----------------------------------------\n')
    
    fprintf('The initial response of $x_a=0$ being negative indicates that the instanteneous centre of rotation is on a line perpendicular to the $x$ axis between $x_a=0$ and $x_a=5$, somewhere above the aircraft. At around $t=1.25, 4$ and $5.75$, all values of $x_a$ experience the same acceleration, indicating that there is no rotation at those times, meaning that the instant centre lies at infinity. Each time this happens, the instant centre switches from above and below the aircraft.')

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.11                 \n')
    fprintf('----------------------------------------\n')
    
    fprintf('As being in the instant centre would not be possible due to it being out of plane, being in line with it would be better. The closer the pilot is to the instant centre, the less g loads the pilot will experience, decreasing the change of fainting spells.')

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.12                 \n')
    fprintf('----------------------------------------\n')
    
    fprintf('By placing the accelerometer close to the nodes of the most important bending mode, the accelerometer experiences reduced oscillations, and has a more accurate reading.')
    
end

if RunQ6
    
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
    fprintf('                             Q6                             \n')
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

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
    
    longitudinal_states_ =[longitudinal_states 13 14];
    PrintStateNames(longitudinal_states_,"longitudinal_states: ")
    lateral_states_ = [lateral_states 13 15 16];
    PrintStateNames(lateral_states     ,"lateral_states: ")
    
    if 1 % Omits Input Dynamics!
        %--- Longitudinal ---
        A_los = A_lo(longitudinal_states_,longitudinal_states_);
        A_long = A_los(1:4,1:4);
        B_long = A_los(1:4,5:6);
        C_los = C_lo(longitudinal_states_,longitudinal_states_);
        C_long = C_los(1:4,1:4);
        D_long = C_los(1:4,5:6);

        %--- Lateral---
        A_las = A_lo(lateral_states_,lateral_states_);
        A_lat = A_las(1:4,1:4);
        B_lat = A_las(1:4,5:6);
        C_las = C_lo(lateral_states_,lateral_states_);
        C_lat = C_las(1:4,1:4);
        D_lat = C_las(1:4,5:6);
        
    else
        %--- Longitudinal ---
        A_long = A_longitude_lo;
        A_lat  = A_lateral_lo;
        B_long = B_longitude_lo;
        B_lat  = B_lateral_lo;
        %--- Lateral---
        C_long = C_longitude_lo;
        C_lat  = C_lateral_lo;
        D_long = D_longitude_lo;
        D_lat  = D_lateral_lo;
    end
    
    %..............................
    % Show Simplified Matrics
    %..............................
    
    if 1
        %--- Longitudinal ---
        A_long
        B_long
        C_long
        D_long
        %--- Lateral---
        A_lat
        B_lat
        C_lat
        D_lat
    end 
    
    %------------------------- Full System --------------------------------
    if 0%1
    
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

        figure(61);
        grid on
        pzmap(tf_long)
        t = title('Full Longitudinal Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)

        figure(62);
        grid on
        pzmap(tf_lat)
        t = title('Full Lateral Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)
    end
    
    %------------------------ Eigen Motions -------------------------------

    fprintf('----------------------------------------\n')
    fprintf('                  Q6.1                  \n')
    fprintf('----------------------------------------\n')

    %..............................
    % Extract Longitudinal Eigen-Motions
    %..............................
    
    %--- Temporary Aliases ---
    % Inputs
    Ut_     = 1;
    Ue_     = 2;
    % Outputs
    Yh_     = 1;
    Ytheta_ = 2;
    Yv_     = 3;
    Yalpha_ = 4;
    
    %--- Short Period + Phugoid ---
    tf_long_Ue_theta       = minreal(tf(C_long(Ytheta_,:) * (inv((s*eye(size(A_long,1))-A_long))*B_long(:,Ue_))),e_minreal)
    tf_long_Ue_theta_poles = esort(pole(tf_long_Ue_theta));
    tf_long_Ue_theta_poles = unique_complex(tf_long_Ue_theta_poles,e)
    
    poles_phugoid     = tf_long_Ue_theta_poles(1:2)
    poles_shortperiod = tf_long_Ue_theta_poles(3:4)
    
    %.. Phugoid ..
    AnalyzePeriodicPoles(poles_phugoid    ,'Phugoid')
    %.. Short Period ..
    AnalyzePeriodicPoles(poles_shortperiod,'Short Period')
        
    %..............................
    % Plot Longitudinal Eigen-Motions
    %..............................
    
    if 0
        figure(63);
        grid on
        pzmap(tf_long_Ue_theta)
        t = title('Longitudinal Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)
    end

    %.. Phugoid ..
    figure(64);
    grid on
    impulse(zpk([],poles_phugoid,1))
    t = title('Phugoid');
    print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)

    %.. Short Period ..
    figure(65);
    grid on
    impulse(zpk([],poles_shortperiod,1))
    t = title('Short Period');
    print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)

    %..............................
    % Extract Lateral Eigen-Motions
    %..............................
    
    % Inputs
    Ut_     = 1;
    Ua_     = 2;
    Ur_     = 3;
    % % Outputs 4 6 7 9 10 12
    Xphi_  = 1;
    Xpsi_  = 2;
    Xv_    = 3;
    Xbeta_ = 4;
    Xp_    = 5;
    Xr_    = 6;

    %--- Aperiodic roll + Spiral ---
    tf_lat_Ua       = minreal(tf(C_lat * (inv((s*eye(size(A_lat,1))-A_lat))*B_lat(:,Ua_))),e_minreal)
    tf_lat_Ua_poles = esort(pole(tf_lat_Ua));
    tf_lat_Ua_poles = unique_complex(tf_lat_Ua_poles,e)
    
    poles_dutchroll     = tf_lat_Ua_poles(3:4)
    pole_aperiodicroll  = tf_lat_Ua_poles(1)
    pole_spiral         = tf_lat_Ua_poles(2)
    
    %.. Dutch roll ..
    AnalyzePeriodicPoles(poles_dutchroll   ,'Dutch roll')
    %.. Aperiodic roll ..
    AnalyzeAperiodicPole(pole_aperiodicroll,'Aperiodic roll')
    %.. Spiral ..
    AnalyzeAperiodicPole(pole_spiral       ,'Spiral')
        
    %..............................
    % Plot Lateral Eigen-Motions
    %..............................
    
    if 1
        figure(66);
        grid on
        pzmap(tf_lat_Ua)
        t = title('Lateral Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)
    end

    %.. Dutch roll ..
    figure(67);
    grid on
    impulse(zpk([],poles_dutchroll,1))
    t = title('Dutch roll');
    print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)

    %.. Aperiodic roll ..
    figure(68);
    grid on
    step(zpk([],pole_aperiodicroll,1))
    t = title('Aperiodic roll');
    print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)
    
    %.. Spiral ..
    figure(69);
    grid on
    step(zpk([],pole_spiral,1))
    t = title('Spiral');
    print(gcf, '-dpng', strcat(figpath,'/',t.String,figext), dpi)

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

function PlotElevatorStepInput(tf_, name)
	opt = stepDataOptions('StepAmplitude', -1);
	T = 0:0.01:6;   
	grid on
	[y,t] = step(tf_, T, opt);
    if (name == '')
	plot(t,y);
    else
	plot(t,y, 'DisplayName',name);
    end
	title('Negative Elevator step input');
	xlabel('Time [s]');
	ylabel('Normal acceleration in z [g]');
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Analyze Pole
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function AnalyzePeriodicPoles(poles,name) 
    fprintf("Analysis of poles %s\n",name)
    pole = poles(2);
    wn     = abs(pole)              % Natural frequency
    dr     = (cos(angle(pole)))     % Dampening ratio
    P      = 2*pi/(wn*sqrt(1-dr^2)) % Period
    T_half = log(2)/(wn*dr)         % Time to damp to half amplitude
end

function AnalyzeAperiodicPole(pole,name) 
    fprintf("Analysis of pole %s\n",name)
    wn     = abs(pole)           % Natural frequency
    dr     = (cos(angle(pole))); % Dampening ratio
    TC     = 1./real(pole)       % Time Constant
    T_half = log(2)/(wn*dr)      % Time to damp to half amplitude
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Unique Complex Array
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function X = unique_complex(X,e)
    [b, ind] = unique(round(X / (e * (1 + i))));
    X = X(ind);
end
