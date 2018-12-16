%{
Created on Tuesday 11.12.2018
Copyright (??) Henricus N. Basien
Author: Henricus N. Basien
Email: Henricus@Basien.de
%}

%****************************************************************************************************
% AE4301P
%****************************************************************************************************

clear all
clc
close all

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
    	ti = title('Ue-YNz Pole-Zero Map');
    	print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
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
    ti = title('xa Shift');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
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
        A_los   = A_lo(longitudinal_states_,longitudinal_states_);
        A_long = A_los(1:4,1:4);
        B_long = A_los(1:4,5:6);
        C_los   = C_lo(longitudinal_states_,longitudinal_states_);
        C_long = C_los(1:4,1:4);
        D_long = C_los(1:4,5:6);

        %.. Temporary Aliases ..
        % Inputs
        Ut_long     = 1;
        Ue_long     = 2;
        % States
        Xv_long     = 1;
        Xalpha_long = 2;
        Xtheta_long = 3;
        Xq_long     = 4;

        %--- Lateral---
        A_las  = A_lo(lateral_states_,lateral_states_);
        A_lat = A_las(1:4,1:4);
        B_lat = A_las(1:4,5:6);
        C_las  = C_lo(lateral_states_,lateral_states_);
        C_lat = C_las(1:4,1:4);
        D_lat = C_las(1:4,5:6);

        %.. Temporary Aliases ..
        % Inputs
        Ut_lat     = 1;
        Ua_lat     = 2;
        Ur_lat     = 2;
        % Outputs
        Ybeta_lat = 1;
        Yphi_lat  = 2;
        Yp_lat    = 3;
        Yr_lat    = 4;
        
    else
        %--- Longitudinal ---
        A_long = A_longitude_lo;
        B_long = B_longitude_lo;
        C_long = C_longitude_lo;
        D_long = D_longitude_lo;

        %.. Temporary Aliases ..
        % Inputs
        Ut_long     = 1;
        Ue_long     = 2;
        % Outputs
        Xh_long     = 1;
        Xtheta_long = 2;
        Xv_long     = 3;
        Xalpha_long = 4;

        %--- Lateral---
        A_lat  = A_lateral_lo;
        B_lat  = B_lateral_lo;
        C_lat  = C_lateral_lo;
        D_lat  = D_lateral_lo;

        %.. Temporary Aliases ..
        % Inputs
        Ut_lat     = 1;
        Ua_lat     = 2;
        Ur_lat     = 3;
        % States
        Xphi_lat  = 1;
        Xpsi_lat  = 2;
        Xv_lat    = 3;
        Xbeta_lat = 4;
        Xp_lat    = 5;
        Xr_lat    = 6;

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
        ti = title('Full Longitudinal Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        figure(62);
        grid on
        pzmap(tf_lat)
        ti = title('Full Lateral Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end
    
    %------------------------ Eigen Motions -------------------------------

    fprintf('----------------------------------------\n')
    fprintf('                  Q6.1                  \n')
    fprintf('----------------------------------------\n')

    %..............................
    % Extract Longitudinal Eigen-Motions
    %..............................
    
    %--- Short Period + Phugoid ---
    tf_long_Ue_theta       = minreal(tf(C_long(Xtheta_long,:) * (inv((s*eye(size(A_long,1))-A_long))*B_long(:,Ue_long))),e_minreal)
    tf_long_Ue_theta_poles = esort(pole(tf_long_Ue_theta));
    tf_long_Ue_theta_poles = unique_complex(tf_long_Ue_theta_poles,e)
    
    poles_phugoid     = tf_long_Ue_theta_poles(1:2)
    poles_shortperiod = tf_long_Ue_theta_poles(3:4)
    
    %.. Phugoid ..
    PrintAnalyzePeriodicPoles(poles_phugoid    ,'Phugoid')
    %.. Short Period ..
    PrintAnalyzePeriodicPoles(poles_shortperiod,'Short Period')
        
    %..............................
    % Plot Longitudinal Eigen-Motions
    %..............................
    
    if 0
        figure(63);
        grid on
        pzmap(tf_long_Ue_theta)
        ti = title('Longitudinal Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end

    %.. Phugoid ..
    figure(64);
    grid on
    impulse(zpk([],poles_phugoid,1))
    ti = title('Phugoid');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    %.. Short Period ..
    figure(65);
    grid on
    impulse(zpk([],poles_shortperiod,1))
    ti = title('Short Period');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    %..............................
    % Extract Lateral Eigen-Motions
    %..............................

    %--- Aperiodic roll + Spiral ---
    tf_lat_Ua       = minreal(tf(C_lat * (inv((s*eye(size(A_lat,1))-A_lat))*B_lat(:,Ua_lat))),e_minreal)
    tf_lat_Ua_poles = esort(pole(tf_lat_Ua));
    tf_lat_Ua_poles = unique_complex(tf_lat_Ua_poles,e)
    
    poles_dutchroll     = tf_lat_Ua_poles(3:4)
    pole_aperiodicroll  = tf_lat_Ua_poles(1)
    pole_spiral         = tf_lat_Ua_poles(2)
    
    %.. Dutch roll ..
    PrintAnalyzePeriodicPoles(poles_dutchroll   ,'Dutch roll')
    %.. Aperiodic roll ..
    PrintAnalyzeAperiodicPole(pole_aperiodicroll,'Aperiodic roll')
    %.. Spiral ..
    PrintAnalyzeAperiodicPole(pole_spiral       ,'Spiral')
        
    %..............................
    % Plot Lateral Eigen-Motions
    %..............................
    
    if 1
        figure(66);
        grid on
        pzmap(tf_lat_Ua)
        ti = title('Lateral Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end

    %.. Dutch roll ..
    figure(67);
    grid on
    impulse(zpk([],poles_dutchroll,1))
    ti = title('Dutch roll');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    %.. Aperiodic roll ..
    figure(68);
    grid on
    step(zpk([],pole_aperiodicroll,1))
    ti = title('Aperiodic roll');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    
    %.. Spiral ..
    figure(69);
    grid on
    step(zpk([],pole_spiral,1))
    ti = title('Spiral');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

end

if RunQ7
    
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
    fprintf('                             Q7                             \n')
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

    FindF16Dynamics
    if 0%1
        tf_long_Ue_q = minreal(tf(C_lo(Yq,:) * (inv((s*eye(size(A_lo,1))-A_lo))*B_lo(:,Ue))),e_minreal)
        tf_long_Ue_q_poles = esort(pole(tf_long_Ue_q));
        tf_long_Ue_q_poles = unique_complex(tf_long_Ue_q_poles,e)

        figure(71)
        pzmap(  tf_long_Ue_q)
        figure(72)
        impulse(tf_long_Ue_q)
        figure(73)
        step(   tf_long_Ue_q)
    end
    
    fprintf('----------------------------------------\n')
    fprintf('                  Q7.1                  \n')
    fprintf('----------------------------------------\n')
    
    %--- Aliases ---
    alphaq_states = [8,11];
    Ue__    = 1;
    Xalha__ = 1;
    Xq__    = 2;

    %--- State space Matrices ---
    PrintStateNames(alphaq_states,"alphaq_states: ")
    A_alphaq = A_lo(alphaq_states,alphaq_states)
    B_alphaq = [0;,-2*pi] %-20.2000]%-1]
    C_alphaq = eye(2)
    D_alphaq = zeros(2,1)

    fprintf('----------------------------------------\n')
    fprintf('                  Q7.2                  \n')
    fprintf('----------------------------------------\n')

    tf_long_Ue_q   = minreal(tf(C_alphaq(Xq__    ,:) * (inv((s*eye(size(A_alphaq,1))-A_alphaq))*B_alphaq(:,Ue__    ))),e_minreal)
    tf_long_Ue_q_4 = minreal(tf(C_long  (Xq_long ,:) * (inv((s*eye(size(A_long  ,1))-A_long  ))*B_long  (:,Ue_long ))),e_minreal)

	T = 0:0.01:7;
    
    figure(74)
    grid on
    [y1,t] = step(tf_long_Ue_q,T);
    plot(t,y1)
    ti = title('2-State Ue-q Step');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    figure(75)
    grid on
    [y2,t] = step(tf_long_Ue_q_4,T);
    plot(t,y2)
    ti = title('4-State Ue-q Step');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    figure(76)
    grid on
    ydiff = y2-y1;
    plot(t,ydiff)
    ti = title('2-4-State Ue-q Step Difference');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    fprintf('----------------------------------------\n')
    fprintf('                  Q7.3                  \n')
    fprintf('----------------------------------------\n')

    tf_long_Ue_q_poles = esort(pole(tf_long_Ue_q));
    tf_long_Ue_q_poles = unique_complex(tf_long_Ue_q_poles,e)

    if 0%1
        figure(77)
        pzmap(  tf_long_Ue_q)
        % figure(78)
        % impulse(tf_long_Ue_q)
    end 
    
    %.. Short Period ..

    CheckShortPeriodDesign(tf_long_Ue_q,tf_long_Ue_q_poles,'Short Period - Original')

    fprintf('----------------------------------------\n')
    fprintf('                  Q7.4                  \n')
    fprintf('----------------------------------------\n')

    %..............................
    % Find Design Poles
    %..............................

    %--- Pole Placement ---

    [wn_design,TC_design,dr_design] = GetShortPeriodDesignCriteria
    a = acos(dr_design);
    real_design = -wn_design*cos(a);
    imag_design =  wn_design*sin(a);

    poles_design = [real_design+1i*imag_design,real_design-1i*imag_design]
    Compensator_1  = zpk(tf_long_Ue_q_poles,poles_design,1)

    %--- Time constant ---
    TC = GetTC(tf_long_Ue_q);
    
    Compensator_2 = (1+s*TC_design)/(1+s*TC)

    %--- Combine ---
    Compensator = Compensator_1*Compensator_2

    %..............................
    % Analyze Design Poles
    %..............................

    figure(78)
    grid on
    [y_design,t] = step(Compensator ,T);
    plot(t,y_design)
    ti = title('Short Period - Compensator');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    if 0%1
        figure(79)
        pzmap(Compensator)
        ti = title('Short Period - Compensator Pole-Zero Map');
    end

    CheckShortPeriodDesign(Compensator,poles_design,'Short Period - Compensator')

    %..............................
    % Apply Compensator
    %..............................
    
    tf_long_Ue_q_design = minreal(tf_long_Ue_q*Compensator,e_minreal)

    tf_long_Ue_q_design_poles = esort(pole(tf_long_Ue_q_design));
    tf_long_Ue_q_design_poles = unique_complex(tf_long_Ue_q_design_poles,e)

    figure(712)
    step(tf_long_Ue_q_design)
    ti = title('Short Period - Design');
    print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

    if 0%1
        figure(713)
        pzmap(tf_long_Ue_q_design)
        ti = title('Short Period - Design Pole-Zero Map');
    end

    CheckShortPeriodDesign(tf_long_Ue_q_design,tf_long_Ue_q_design_poles,'Short Period - Design')

    fprintf('----------------------------------------\n')
    fprintf('                  Q7.5                  \n')
    fprintf('----------------------------------------\n')
    
    fprintf('The lead-lag prefilter must be located outside the loop, as it has to modify the zero of the closed loop system. The zero that is being forced has no place inside the denominator of the overal transfer function, since that changes the dynamics. Instead, it is outside the loop to modify the reference signal to something that the closed loop can quickly follow. \n')
       
    fprintf('----------------------------------------\n')
    fprintf('                  Q7.6                  \n')
    fprintf('----------------------------------------\n')

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

function [wn,dr,P,T_half] = AnalyzePeriodicPoles(poles) 
    pole = poles(1);
    wn     = abs(pole);              % Natural frequency
    dr     = -(cos(angle(pole)));    % Dampening ratio
    P      = 2*pi/(wn*sqrt(1-dr^2)); % Period
    T_half = log(2)/(wn*dr);         % Time to damp to half amplitude
end

function PrintAnalyzePeriodicPoles(poles,name)
    fprintf("Analysis of Periodic pole %s\n",name) 
    [wn,dr,P,T_half] = AnalyzePeriodicPoles(poles)

    fprintf('wn    : %f \n',wn)
    fprintf('dr    : %f \n',dr)
    fprintf('P     : %f \n',P)
    fprintf('T_half: %f \n',T_half)    
end

function [wn,dr,TC,T_half] = AnalyzeAperiodicPole(pole) 
    wn     = abs(pole);           % Natural frequency
    dr     = (cos(angle(pole))); % Dampening ratio
    TC     = 1./real(pole);       % Time Constant
    T_half = log(2)/(wn*dr);      % Time to damp to half amplitude
end

function PrintAnalyzeAperiodicPole(pole,name)
    fprintf("Analysis of Aperiodic pole %s\n",name) 
    [wn,dr,TC,T_half] = AnalyzeAperiodicPole(pole)

    fprintf('wn    : %f \n',wn)
    fprintf('dr    : %f \n',dr)
    fprintf('TC    : %f \n',TC)
    fprintf('T_half: %f \n',T_half)    
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Unique Complex Array
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function X = unique_complex(X,e)
    [b, ind] = unique(round(X / (e * (1 + i))));
    X = X(ind);
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Get TC
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function TC = GetTC(tf)
    [num_, den_] = tfdata(tf);
    num = num_{1};
    kq  = num(3);
    TC  = num(2)/kq; 
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Check Short Period Design Criteria
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function [wn_design,TC_design,dr_design] = GetShortPeriodDesignCriteria()
    %----------------------------------------
    % Get Design Criteria
    %----------------------------------------
    global velocity0 USE_SI_UNITS feet_to_m
    v = velocity0;
    if ~USE_SI_UNITS
        v = v*feet_to_m; 
    end
    
    wn_design = 0.03*v;
    TC_design = 1./0.75*wn_design;
    dr_design = 0.5;
end

function CheckShortPeriodDesign(sys,poles,name)

    [wn_design,TC_design,dr_design] = GetShortPeriodDesignCriteria;

    %----------------------------------------
    % Analyze System
    %----------------------------------------
    
    [wn,dr,P,T_half] = AnalyzePeriodicPoles(poles);
    TC = GetTC(sys);

    %----------------------------------------
    % Compare
    %----------------------------------------
    
    wn_diff = wn_design-wn;
    TC_diff = TC_design-TC;
    dr_diff = dr_design-dr;

    %----------------------------------------
    % Print differences
    %----------------------------------------
    fprintf('wn_org   : %f \n',wn)
    fprintf('TC_org   : %f \n',TC)
    fprintf('dr_org   : %f \n',dr)
    fprintf('wn_design: %f \n',wn_design)
    fprintf('TC_design: %f \n',TC_design)
    fprintf('dr_design: %f \n',dr_design)
    fprintf('wn_diff  : %f \n',wn_diff  )
    fprintf('TC_diff  : %f \n',TC_diff  )
    fprintf('dr_diff  : %f \n',dr_diff  )
end

function drawCAP(wn, dr, TC)
   CAP =  g0 * wn^2 * TC / velocity0;
end
