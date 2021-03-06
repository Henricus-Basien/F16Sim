%{
Created on Tuesday 11.12.2018
Copyright (??) Henricus N. Basien
Author: Henricus N. Basien
        Raoul A. A. Mink
Email: Henricus@Basien.de
%}

%****************************************************************************************************
% AE4301P
%****************************************************************************************************

clear all
clc
close all
bdclose('all')

%================================================================================
% Settings
%================================================================================

Settings
StateAliases

%================================================================================
% Excercises
%================================================================================

%input('Press ENTER to run Full Analysis...');

%...........................................................................................................................

if RunQ5
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
    fprintf('                             Q5                             \n')
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.2                  \n')
    fprintf('----------------------------------------\n')

    fprintf('Trimming Linearized Model\n')
    altitude0 = 15000 % [ft]
    velocity0 = 500   % [ft/s]

    if USE_SI_UNITS
        altitude0 = altitude0 * feet_to_m;
        velocity0 = velocity0 * feet_to_m;
    end

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

    Nz_index = find(C_Nz,NrOutputs);
    PrintStateNames(Nz_index,'Elements Nz depends on: ')

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.5                  \n')
    fprintf('----------------------------------------\n')

    tf_Ue_Nz = minreal(tf(C_lo(YNz,:) * (inv((s*eye(NrStates)-A_lo))*B_lo(:,Ue))),e_minreal)

    fprintf('----------------------------------------\n')
    fprintf('                  Q5.6                  \n')
    fprintf('----------------------------------------\n')

    fprintf('Negative Elevator step input output Nz\n');

    if PlotQ5
        figure(51);
        PlotElevatorStepInput(tf_Ue_Nz, '')
        grid on
        ti = title('Neg. Elevator step input - n_a');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end
        
    fprintf('----------------------------------------\n')
    fprintf('                  Q5.7                  \n')
    fprintf('----------------------------------------\n')

    tf_Ue_Nz_zeros = zero(tf_Ue_Nz)
    tf_Ue_Nz_poles = pole(tf_Ue_Nz)

    if PlotQ5
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

    if PlotQ5
        figure(53);  
        for xa = xa_
        
            fprintf('Analyzing xa: %f %s\n',xa,lu)
            if 0%1
                FindF16Dynamics
            else
                [A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);
                %C_lo
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
        hold off
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        xlim([0 1]);
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,'_zoom',figext), dpi)
        
    end

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

%...........................................................................................................................

if RunQ6
    
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
    fprintf('                             Q6                             \n')
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

    altitude0 = altitude00 % [ft]
    velocity0 = velocity00 % [ft/s]

    if USE_SI_UNITS
        altitude0 = altitude0 * feet_to_m;
        velocity0 = velocity0 * feet_to_m;
    end

    FindF16Dynamics
    if ApplyStateSpaceSimplification == 1
        SimplifyStatespace
    end
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
    
    longitudinal_states_ = [longitudinal_states 13 14];
    PrintStateNames(longitudinal_states,"longitudinal_states: ")
    lateral_states_ = [lateral_states 15 16];
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
        Yv_long     = 1;
        Yalpha_long = 2;
        Ytheta_long = 3;
        Yq_long     = 4;

        %--- Lateral---
        A_las  = A_lo(lateral_states_,lateral_states_);
        A_lat = A_las(1:4,1:4);
        B_lat = A_las(1:4,5:6);
        C_las  = C_lo(lateral_states_,lateral_states_);
        C_lat = C_las(1:4,1:4);
        D_lat = C_las(1:4,5:6);

        %.. Temporary Aliases ..
        % Inputs
        Ua_lat     = 1;
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
        Yh_long     = 1;
        Ytheta_long = 2;
        Yv_long     = 3;
        Yalpha_long = 4;

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
        Yphi_lat  = 1;
        Ypsi_lat  = 2;
        Yv_lat    = 3;
        Ybeta_lat = 4;
        Yp_lat    = 5;
        Yr_lat    = 6;

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
    if 1
    
        %..............................
        % Get Transfer Functions
        %..............................

        tf_long = minreal(tf(C_long * (inv((s*eye(size(A_long,1))-A_long))*B_long)),e_minreal)
        tf_lat  = minreal(tf(C_lat  * (inv((s*eye(size(A_lat ,1))-A_lat ))*B_lat )),e_minreal)
        
        %..............................
        % Get Poles & Zeros
        %..............................

        %--- Longitudinal ---
        %.. Zeros ..
        tf_long_zeros = esort(zero(tf_long));
        tf_long_zeros = unique_complex(tf_long_zeros,e)
        %.. Poles ..
        tf_long_poles = esort(pole(tf_long));
        tf_long_poles = unique_complex(tf_long_poles,e)
        %--- Lateral ---
        %.. Zeros ..
        tf_lat_zeros  = esort(zero(tf_lat));
        tf_lat_zeros  = unique_complex(tf_lat_zeros,e)
        %.. Poles ..
        tf_lat_poles  = esort(pole(tf_lat));
        tf_lat_poles  = unique_complex(tf_lat_poles,e)

        %..............................
        % Plot Pole-Zero maps
        %..............................
        if PlotQ6
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
    end
    
    %------------------------ Eigen Motions -------------------------------

    fprintf('----------------------------------------\n')
    fprintf('                  Q6.1                  \n')
    fprintf('----------------------------------------\n')

    %..............................
    % Extract Longitudinal Eigen-Motions
    %..............................
    
    %--- Short Period + Phugoid ---
    tf_long_Ue_v     = tf_long(Yv_long    ,Ue_long)
    tf_long_Ue_theta = tf_long(Ytheta_long,Ue_long)
    tf_long_Ue_q     = tf_long(Yq_long    ,Ue_long)
    
    poles_phugoid     = tf_long_poles(1:2);
    poles_shortperiod = tf_long_poles(3:4);
    
    %.. Phugoid ..
    PrintAnalyzePeriodicPoles(poles_phugoid    ,'Phugoid')
    %.. Short Period ..
    PrintAnalyzePeriodicPoles(poles_shortperiod,'Short Period')
        
    %..............................
    % Plot Longitudinal Eigen-Motions
    %..............................
    
    %.. Phugoid ..
    sys_phugoid_dominant = zpk([],poles_phugoid,1);

    if PlotQ6
        T = 0:0.01:600;  

        figure(63);
        grid on
        hold on
        xlabel('Time [s]')

        [y,t] = impulse(tf_long_Ue_v    ,T);
        if ~USE_SI_UNITS
            y = y*feet_to_m;
        end
        plot(t,y,'DisplayName','Impulse Response: \delta_e - V')
        ylabel('Velocity [m/s]')
        yyaxis right
        [y,t] = impulse(tf_long_Ue_theta,T);
        plot(t,y,'DisplayName','Impulse Response: \delta_e - \theta')
        ylabel('\theta [\circ]')
        % [y,t] = impulse(tf_long_Ue_theta,T);
        % plot(t,y,'DisplayName','Full System - Impulse Response')
        % if ShowDominantPoles
        %     [y,t] = impulse(sys_phugoid_dominant,T);
        %     plot(t,y,'DisplayName','Dominant poles - Impulse Response')
        % end
        % ylabel('Impulse Response')
        % yyaxis right
        % [y,t] = step(tf_long_Ue_theta,T);
        % plot(t,y,'DisplayName','Full System - Step Response')
        % if ShowDominantPoles
        %     [y,t] = step(sys_phugoid_dominant,T);
        %     plot(t,y,'DisplayName','Dominant poles - Step Response')
        % end
        % ylabel('Step Response')
        
        ti = title('Phugoid');
        legend('Location','southeast')
        hold off
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end
    
    %.. Short Period ..
    sys_shortperiod_dominant = zpk([],poles_shortperiod,1);
    if PlotQ6
        T = 0:0.01:7;  

        figure(64);
        grid on
        hold on
        xlabel('Time [s]')

        [y,t] = impulse(tf_long_Ue_theta,T);
        plot(t,y,'DisplayName','Impulse Response: \delta_e - \theta')
        ylabel('\theta [\circ]')
        yyaxis right
        [y,t] = impulse(tf_long_Ue_q,T);
        plot(t,y,'DisplayName','Impulse Response: \delta_e - q')
        [y,t] = step(tf_long_Ue_q,T);
        plot(t,y,'DisplayName','Step Response: \delta_e - q')
        ylabel('Pitch Rate [\circ/s]')
        % [y,t] = impulse(tf_long_Ue_theta,T);
        % plot(t,y,'DisplayName','Full System - Impulse Response')
        % if ShowDominantPoles
        %     [y,t] = impulse(sys_shortperiod_dominant,T);
        %     plot(t,y,'DisplayName','Dominant poles - Impulse Response')
        % end
        % ylabel('Impulse Response - \theta [\circ]')
        % yyaxis right
        % [y,t] = step(tf_long_Ue_theta,T);
        % plot(t,y,'DisplayName','Full System - Step Response')
        % if ShowDominantPoles
        %     [y,t] = step(sys_shortperiod_dominant,T);
        %     plot(t,y,'DisplayName','Dominant poles - Step Response')
        % end
        % ylabel('Step Response - \theta [\circ]')
        
        ti = title('Short Period');
        legend('Location','southeast')
        hold off
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end

    %..............................
    % Extract Lateral Eigen-Motions
    %..............................

    %--- Aperiodic roll + Spiral ---
    tf_lat_Ur_p = tf_lat(Yp_lat,Ur_lat)
    tf_lat_Ur_r = tf_lat(Yr_lat,Ur_lat)
    tf_lat_Ua_p = tf_lat(Yp_lat,Ua_lat)
    tf_lat_Ua_r = tf_lat(Yr_lat,Ua_lat)

    poles_dutchroll     = tf_lat_poles(3:4);
    pole_aperiodicroll  = tf_lat_poles(1);
    pole_spiral         = tf_lat_poles(2);
    
    %.. Dutch roll ..
    PrintAnalyzePeriodicPoles(poles_dutchroll   ,'Dutch roll')
    %.. Aperiodic roll ..
    PrintAnalyzeAperiodicPole(pole_aperiodicroll,'Aperiodic roll')
    %.. Spiral ..
    PrintAnalyzeAperiodicPole(pole_spiral       ,'Spiral')
        
    %..............................
    % Plot Lateral Eigen-Motions
    %..............................
    
    %.. Dutch roll ..
    if PlotQ6
        T = 0:0.01:10;  

        figure(651);
        grid on
        hold on
        xlabel('Time [s]')

        %impulse(zpk([],poles_dutchroll,1))
        [yp,t] = impulse(tf_lat_Ur_p,T);
        plot(t,yp,'DisplayName','Impulse Response \delta_r - Roll Rate')
        [yr,t] = impulse(tf_lat_Ur_r,T);
        plot(t,yr,'DisplayName','Impulse Response \delta_r - Yaw Rate')

        ylabel('Angular Rate [\circ/s')

        ti = title('Dutch roll');
        legend('Location','southeast')
        hold off
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        figure(652)
        plot(yp,yr)
        xlabel('Roll Rate [\circ/s]')
        ylabel('Yaw Rate [\circ/s]')
        ti = title('Dutch roll - Rates');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end
    %.. Aperiodic roll ..
    if PlotQ6
        figure(66);
        grid on
        hold on
        xlabel('Time [s]')

        %step(zpk([],pole_aperiodicroll,1));
        [y,t] = impulse(tf_lat_Ua_p);
        plot(t,y,'DisplayName','Impulse Response: \delta_a - p')
        % [y,t] = step(tf_lat_Ua_p);
        % plot(t,y,'DisplayName','Step Response: \delta_a - p')
        ylabel('Roll Rate [\circ/s]')

        hold off
        ti = title('Aperiodic roll');
        legend('Location','southeast')
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end
    %.. Spiral ..
    if PlotQ6
        T = 0:0.01:90;%180;  

        figure(67);
        grid on
        hold on
        xlabel('Time [s]')

        %step(zpk([],pole_spiral,1));
        [y,t] = impulse(tf_lat_Ua_r,T);
        plot(t,y,'DisplayName','Impulse Response: \delta_a - r')
        % [y,t] = step(tf_lat_Ua_r,T);
        % plot(t,y,'DisplayName','Step Response - r')
        ylabel('Yaw Rate [\circ/s]')

        hold off
        ti = title('Spiral');
        legend('Location','southeast')
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end

end

%...........................................................................................................................

if RunQ7
    
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
    fprintf('                             Q7                             \n')
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

    FindF16Dynamics
    if ApplyStateSpaceSimplification == 1
        SimplifyStatespace
    end
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

    Ue__     = 1;
    Yalpha__ = 1;
    Yq__     = 2;
    reduce2_states = [2 4];
    reduce2_inputs = [2];
    A_2state = A_long(reduce2_states,reduce2_states)
    B_2state = B_long(reduce2_states,reduce2_inputs)
    C_2state = C_long(reduce2_states,reduce2_states)
    D_2state = D_long(reduce2_states,reduce2_inputs)
    A_alphaq = A_2state;
    B_alphaq = B_2state;
    C_alphaq = C_2state;
    D_alphaq = D_2state;
        
    fprintf('----------------------------------------\n')
    fprintf('                  Q7.2                  \n')
    fprintf('----------------------------------------\n')

    tf_long_Ue_q   = minreal(tf(C_alphaq(Yq__    ,:) * (inv((s*eye(size(A_alphaq,1))-A_alphaq))*B_alphaq(:,Ue__    ))),e_minreal)
    tf_long_Ue_q_4 = minreal(tf(C_long  (Yq_long ,:) * (inv((s*eye(size(A_long  ,1))-A_long  ))*B_long  (:,Ue_long ))),e_minreal)

    T = 0:0.01:7;
    
    if PlotQ7
        figure(74)
        hold on
        %grid on
        [y1,t] = step(tf_long_Ue_q,T);
        plot(t,y1)
        xlabel('Time [s]');ylabel('Pitch Rate [deg/s]');

        [y2,t] = step(tf_long_Ue_q_4,T);
        plot(t,y2)
        xlabel('Time [s]');ylabel('Pitch Rate [deg/s]');
        ti = title('n-State Ue-q Step');
        legend('2-State', '4-State')

        hold off
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        
        figure(76)
        grid on
        ydiff = y2-y1;
        plot(t,ydiff)
        xlabel('Time [s]');ylabel('Pitch Rate [deg/s]');
        ti = title('2-4-State Ue-q Step Difference');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end
        
    fprintf('----------------------------------------\n')
    fprintf('                  Q7.3                  \n')
    fprintf('----------------------------------------\n')

    tf_long_Ue_q_poles = esort(pole(tf_long_Ue_q));
    tf_long_Ue_q_poles = unique_complex(tf_long_Ue_q_poles,e)

    if 0%PlotQ7
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

    [wn_design,T_theta_design,dr_design] = GetShortPeriodDesignCriteria
    a = acos(dr_design);
    real_design = -wn_design*cos(a);
    imag_design =  wn_design*sin(a);

    poles_design = [real_design+1i*imag_design,real_design-1i*imag_design]
    Compensator_1  = zpk(tf_long_Ue_q_poles,poles_design,1)
    
    K = place(A_alphaq, B_alphaq, poles_design);
    kq = K(2);
    kalpha = K(1);
    [y1 t] = step(tf_long_Ue_q);
    [y2 t] = step(tf_long_Ue_q * Compensator_1);
    KComp = y1(end)/y2(end)
    Compensator_1 = Compensator_1 * KComp
    
    %--- T theta ---
    T_theta = GetT_theta(tf_long_Ue_q*Compensator_1);
    
    Compensator_2 = (1+s*T_theta_design)/(1+s*T_theta)

    %--- Combine ---
    Compensator = minreal(Compensator_1*Compensator_2,e_minreal)

    %..............................
    % Analyze Design Poles
    %..............................

    if PlotQ7
        figure(78)
        grid on
        step(Compensator ,T);
        ti = title('Short Period - Compensator');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        % figure(79)
        % pzmap(Compensator)
        % ti = title('Short Period - Compensator Pole-Zero Map');
    end

    CheckShortPeriodDesign(Compensator,poles_design,'Short Period - Compensator')

    %..............................
    % Apply Compensator
    %..............................
    
    tf_long_Ue_q_design = minreal(tf_long_Ue_q*Compensator,e_minreal)

    tf_long_Ue_q_design_poles = esort(pole(tf_long_Ue_q_design));
    tf_long_Ue_q_design_poles = unique_complex(tf_long_Ue_q_design_poles,e)

    if PlotQ7
        figure(712)
        step(tf_long_Ue_q_design)
        ti = title('Short Period - Design');
        xlabel('Time [s]');ylabel('Pitch Rate [deg/s]');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        % figure(713)
        % pzmap(tf_long_Ue_q_design)
        % ti = title('Short Period - Design Pole-Zero Map');
    end

    CheckShortPeriodDesign(tf_long_Ue_q_design,tf_long_Ue_q_design_poles,'Short Period - Design')

    % tf_long = minreal(tf(C_alphaq * (inv((s*eye(size(A_alphaq,1))-A_alphaq))*B_alphaq)),e_minreal)
    % tf_long_design = tf_long * Compensator

    %..............................
    % Check Gains
    %..............................
    
    T_theta = GetT_theta(tf_long_Ue_q * Compensator_1);

    tf_long_Ue_alpha        = minreal(tf(C_alphaq(Yalpha__,:) * (inv((s*eye(size(A_alphaq,1))-A_alphaq))*B_alphaq(:,Ue__))),e_minreal)
    tf_long_Ue_alpha_design = minreal(tf_long_Ue_alpha * Compensator_1,e_minreal)

    T_theta = GetT_theta(tf_long_Ue_alpha_design);
    
    %..............................
    % Gust Case
    %..............................
    
    if 0
        v_gust = 4.572; % [m/s]

        if ~USE_SI_UNITS
            v_gust = v_gust/feet_to_m;
        end

        d_alpha        = atan(v_gust/velocity0); % [rad]
        d_alpha_deg    = d_alpha*180/pi
        d_elevator     = d_alpha_deg/kalpha; %kalpha*d_alpha
        d_elevator_deg = d_elevator*180/pi
        
        if PlotQ7
            figure(714)
            opt = stepDataOptions('StepAmplitude', d_elevator);
            T = 0:0.01:10;  
            step(tf_long_Ue_q_design,T,opt)
            ti = title('Short Period - Gust');
            print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        end
    end
    
    fprintf('----------------------------------------\n')
    fprintf('                  Q7.5                  \n')
    fprintf('----------------------------------------\n')
    
    fprintf('The lead-lag prefilter must be located outside the loop, as it has to modify the zero of the closed loop system. The zero that is being forced has no place inside the denominator of the overal transfer function, since that changes the dynamics. Instead, it is outside the loop to modify the reference signal to something that the closed loop can quickly follow. \n')
       
    fprintf('----------------------------------------\n')
    fprintf('                  Q7.6                  \n')
    fprintf('----------------------------------------\n')

    %-------
    
    [CAP ,dr ,DBqss ,qmqs ] = GetCAP_and_Gibson(tf_long_Ue_q_design,tf_long_Ue_q_design_poles)
    [CAP0,dr0,DBqss0,qmqs0] = GetCAP_and_Gibson(tf_long_Ue_q,tf_long_Ue_q_poles)

    dCAP    = [dr    , CAP ]-[dr0   , CAP0 ]
    dGibson = [DBqss , qmqs]-[DBqss0, qmqs0]

    if PlotQ7 %Plots of CAP criteria
        figure(761)
        subplot(2,2,1)
        hold on
        grid on
        title('Flight Phase Category A');
        set(gca, 'XScale', 'log');
        set(gca, 'YScale', 'log');
        xlim([0.01 10]);
        ylim([0.01 10]);
        loglog([0.13 0.13],[0.01 10], 'k');
        rectangle('Position', [0.25 0.16 1.75 9.84]);
        rectangle('Position', [0.3 0.28 0.9 3.32]);
        quiver(dr0, CAP0,dCAP(1),dCAP(2),0)
        loglog(dr , CAP , "x");
        loglog(dr0, CAP0, ".");
        hold off
        
        subplot(2,2,2)
        hold on
        grid on
        title('Flight Phase Category B');
        set(gca, 'XScale', 'log');
        set(gca, 'YScale', 'log');
        xlim([0.01 10]);
        ylim([0.01 10]);
        loglog([0.15 0.15],[0.01 10], 'k');
        rectangle('Position', [0.2 0.038 1.8 9.962]);
        rectangle('Position', [0.3 0.085 1.7 3.515]);
        quiver(dr0, CAP0,dCAP(1),dCAP(2),0)
        loglog(dr , CAP , "x");
        loglog(dr0, CAP0, ".");
        hold off
        
        subplot(2,2,3)
        hold on
        grid on
        title('Flight Phase Category C');
        set(gca, 'XScale', 'log');
        set(gca, 'YScale', 'log');
        xlim([0.01 10]);
        ylim([0.01 10]);
        loglog([0.15 0.15],[0.01 10], 'k');
        rectangle('Position', [0.25 0.005 1.75 9.985]);
        rectangle('Position', [0.35 0.16 0.95 3.44]);
        quiver(dr0, CAP0,dCAP(1),dCAP(2),0)
        loglog(dr , CAP , "x");
        loglog(dr0, CAP0, ".");
        hold off
    end
    
    if PlotQ7 %Plots of Gibson Criteria
       subplot(2,2,4)
       hold on
       grid on
       title('Gibson Criterion');
       v = [0 1; 0 3; 0.06 3; 0.3 1];
       f = [1 2 3 4];
       patch('Faces', f, 'Vertices', v, 'FaceColor', 'none');
       quiver(DBqss0, qmqs0,dGibson(1),dGibson(2),0)
       plot(DBqss , qmqs , "x");
       plot(DBqss0, qmqs0, ".");
       
       hold off

       print(gcf, '-dpng', strcat(figpath,'/','Pitch Rate Criteria',figext), dpi)
    end
    
    if PlotQ7 %Time response of pitch rate and pitch attitude
        opt = stepDataOptions('StepAmplitude', -1);

        figure(765)
        step(tf_long_Ue_q_design, T, opt); %pitch rate
        ti = title('Pitch rate step response');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        figure(766)
        step(tf_long_Ue_q_design*(1/s), T, opt); %pitch attitude
        ti = title('Pitch attitude step response');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end
end

%...........................................................................................................................

if RunQ8
    
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
    fprintf('                             Q8                             \n')
    fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

    fprintf('----------------------------------------\n')
    fprintf('                  Q8.1                  \n')
    fprintf('----------------------------------------\n')

    altitude0 = 5000; % [ft]
    velocity0 = 300;  % [ft/s]

    if USE_SI_UNITS
        altitude0 = altitude0 * feet_to_m;
        velocity0 = velocity0 * feet_to_m;
    end

    FindF16Dynamics
    if ApplyStateSpaceSimplification == 1
        SimplifyStatespace
    end

    theta0 = xu_lo(5)
    alpha0 = xu_lo(8)
    q0     = xu_lo(11)

    thrust0   = xu_lo(13)
    elevator0 = xu_lo(14)

    fprintf('----------------------------------------\n')
    fprintf('                  Q8.2                  \n')
    fprintf('----------------------------------------\n')

    terrainfollow_inputs = [1,2];
    terrainfollow_states = [3,7,8,5,11];
    PrintStateNames(terrainfollow_states,"TerrainFollowing States: ")
    
    terrainfollow_states_ = [terrainfollow_states,13,14];
    n_inputs = size(terrainfollow_inputs,2);
    n_states = size(terrainfollow_states,2);
    if 0%1
        A_terrainfollow = A_lo(terrainfollow_states_,terrainfollow_states_)
        B_terrainfollow = B_lo(terrainfollow_states_,terrainfollow_inputs)
        C_terrainfollow = C_lo(terrainfollow_states_,terrainfollow_states_)
        D_terrainfollow = D_lo(terrainfollow_states_,terrainfollow_inputs)
    else
        A_temp = A_lo(terrainfollow_states_,terrainfollow_states_);
        A_terrainfollow = A_temp(1:n_states,1:n_states)
        B_terrainfollow = A_temp(1:n_states,n_states+1:n_states+n_inputs)
        C_temp = C_lo(terrainfollow_states_,terrainfollow_states_);
        C_terrainfollow = C_temp(1:n_states,1:n_states)
        D_terrainfollow = C_temp(1:n_states,n_states+1:n_states+n_inputs)
    end

    % ConvertToLaTeX(A_terrainfollow)
    % ConvertToLaTeX(B_terrainfollow)
    % ConvertToLaTeX(C_terrainfollow)
    % ConvertToLaTeX(D_terrainfollow)

    C_terrainfollow_inv = inv(C_terrainfollow);
    
    fprintf('----------------------------------------\n')
    fprintf('                  Q8.3                  \n')
    fprintf('----------------------------------------\n')
    
    fprintf('----------------------------------------\n')
    fprintf('                  Q8.4                  \n')
    fprintf('----------------------------------------\n')
    
    h_terrainfollowing0 = 1500 % [m]
    if ~USE_SI_UNITS
        h_terrainfollowing0 = h_terrainfollowing0/feet_to_m
    end

    dh0 = h_terrainfollowing0-altitude0;

    x_terrainfollow0    = zeros(1,n_states); %[dh0,0,0,0,0 , 0,0]
    x_terrainfollow0(1) = dh0;

    fprintf('----------------------------------------\n')
    fprintf('                  Q8.5                  \n')
    fprintf('----------------------------------------\n')
    
    fprintf('----------------------------------------\n')
    fprintf('                  Q8.6                  \n')
    fprintf('----------------------------------------\n')
    
    fprintf('----------------------------------------\n')
    fprintf('                  Q8.7                  \n')
    fprintf('----------------------------------------\n')

    %..............................
    % Design Controller
    %..............................

    Q = eye(size(A_terrainfollow,1));
    R = eye(size(B_terrainfollow,2));

    %--- Set Design Gains ---
    Q(1,1) = 1.8; % h
    Q(2,2) = 0.17; % v
    Q(3,3) = 0.0001; % alpha
    Q(4,4) = 0.00001; % theta
    Q(5,5) = 0.01; % q

    R(1,1) = 0.005; % thrust
    R(2,2) = 0.05; % elevator

    %--- Show Matrices ---
    Q
    R

    %--- Calculate LQR Gains ---
    [K,S,e] = lqr(A_terrainfollow,B_terrainfollow,Q,R);
    K
    
    %----------------------------------------
    % Manual Control
    %----------------------------------------

    %..............................
    % Run Simulation
    %..............................
    
    UseOptimalControl = 0;
    sim('TerrainFollowing')
    
    %..............................
    % Evaluate Simulation
    %..............................
    
    t_m   = TerrainFollowing_Output_Altitude.time;
    [x_m] = TerrainFollowing_Output_xpos.signals.values;
    [altitude_m,altitude_ref_m,altitude_ground_m] = TerrainFollowing_Output_Altitude.signals.values;
    [altitude_err_m,height_over_ground_m]         = TerrainFollowing_Output_AltitudeError.signals.values;
    [thrust_inputs_m]                             = TerrainFollowing_Output_Thrust.signals.values;
    [elevator_inputs_m]                           = TerrainFollowing_Output_Elevator.signals.values;
    [cost_total_m,cost_average_m]                 = TerrainFollowing_Output_Cost.signals.values;
    
    %----------------------------------------
    % Optimal Control
    %----------------------------------------
    
    UseOptimalControl = 1;
    sim('TerrainFollowing')
    
    %..............................
    % Evaluate Simulation
    %..............................
    
    t   = TerrainFollowing_Output_Altitude.time;
    [x] = TerrainFollowing_Output_xpos.signals.values;
    [altitude,altitude_ref,altitude_ground] = TerrainFollowing_Output_Altitude.signals.values;
    [altitude_err,height_over_ground]       = TerrainFollowing_Output_AltitudeError.signals.values;
    [thrust_inputs]                         = TerrainFollowing_Output_Thrust.signals.values;
    [elevator_inputs]                       = TerrainFollowing_Output_Elevator.signals.values;
    [cost_total,cost_average]               = TerrainFollowing_Output_Cost.signals.values;

    %--- Minimum Safety margin Criterion ---
    d_alt_crit = 20 % [m]

    dalt_min = min(height_over_ground)
    if dalt_min<d_alt_crit
       fprintf('WARNING: The height over ground is <%f m, please reconsider your design!\n',d_alt_crit) 
    end

    %--- Maximum Overshoot Criterion ---
    c_overshoot_max = 1.0 % [m]

    Constant_ref = false;
    Ref_reached = false;
    Pre_ref = 0;

    for i = 1:(numel(t)-1)
        %.. Constant Reference? ..
        if altitude_ref(i)-altitude_ref(i+1) == 0
            %fprintf('Constant Altitude %f\n',altitude_ref(i))
            if ~Constant_ref
                Ref_reached = false;
                Pre_ref = altitude_err(i);
            end
            Constant_ref = true;
        else
            Constant_ref = false;
        end
        if Constant_ref
            if ~Ref_reached
                if ~ (sign(altitude_err(i)) - sign(Pre_ref) == 0)
                    Ref_reached = true;
                    fprintf('Constant Reference Reached @ t=%f \n',t(i));
                end
            else
                if abs(altitude_err(i))>c_overshoot_max
                    fprintf('WARNING: The constant reference overshoot is %f>%f m (@t=%f), please reconsider your design!\n',altitude_err(i),c_overshoot_max,t(i))
                end
            end
        end
    end
    
    %..............................
    % Plot Simulation
    %..............................
    
    if PlotQ8
        %--- Altitude ---
        figure(81)
        hold on
        plot(x,altitude       , 'DisplayName','Altitude (LQR)'        )
        plot(x_m,altitude_m   , 'DisplayName','Altitude (PID)'  )
        plot(x,altitude_ref   , 'DisplayName','Reference Height')
        plot(x,altitude_ground, 'DisplayName','Ground Height'   )
        legend('Location','southeast')
        ti = title('TerrainFollowing - Flightpath');
        xlabel('x-Position [m]')%('Time [s]');
        ylabel('Altitude [m]');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        hold off

        %--- Altitude-Error ---
        figure(82)
        hold on
        plot(x  ,height_over_ground  , 'DisplayName','Height (LQR)'      )
        plot(x_m,height_over_ground_m, 'DisplayName','Height (PID)')
        yline(d_alt_crit,'r', 'DisplayName','Critical Height');
        yline(40-c_overshoot_max,'-.m', 'DisplayName','Minimum Overshoot');
        yline(40                ,'g'  , 'DisplayName','Constant Ref. Height');
        yline(40+c_overshoot_max,'-.m', 'DisplayName','Maximum Overshoot');    
        ti = title('TerrainFollowing - Height over Ground');
        xlabel('x-Position [m]')%('Time [s]');
        ylabel('Height [m]');
        ylim([d_alt_crit-1,max(height_over_ground)+1])
        legend('Location','southeast')
        hold off
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        %--- Inputs ---
        figure(83)
        hold on
        plot(t  ,thrust_inputs    +thrust0  , 'DisplayName','Thrust (LQR)'  )
        plot(t_m,thrust_inputs_m  +thrust0  , 'DisplayName','Thrust (PID)'  )
        ylabel('Thrust [' + fu + ']');
        yyaxis right
        plot(t  ,elevator_inputs  +elevator0, 'DisplayName','Elevator (LQR)')
        plot(t_m,elevator_inputs_m+elevator0, 'DisplayName','Elevator (PID)')
        ylabel('Elevator [deg]'); ylim([elevator_min*1.05 elevator_max*1.05]);
        legend('Location','southeast')
        ti = title('TerrainFollowing - Control Inputs');
        xlabel('Time [s]');
        
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        hold off

        %--- Cost ---
        figure(84)
        hold on
        plot(t  ,cost_total  , 'DisplayName','Cost Total (LQR)')
        plot(t_m,cost_total_m, 'DisplayName','Cost Total (PID)')
        ylabel('Integral Reference Offset (Cost) [m*s]');
        yyaxis right
        plot(t  ,cost_average  , 'DisplayName','Cost Average (LQR)')
        plot(t_m,cost_average_m, 'DisplayName','Cost Average (PID)')
        ylabel('Average Reference Offset [m]');
        legend('Location','southeast')
        ti = title('TerrainFollowing - Cost');
        xlabel('Time [s]');
        
        
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        hold off

    end

end

%================================================================================
% Addendum
%================================================================================

input('Press ENTER to close the Analysis...');
close all
bdclose('all')

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
    disp(poles)
    [wn,dr,P,T_half] = AnalyzePeriodicPoles(poles);

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
    disp(pole)
    [wn,dr,TC,T_half] = AnalyzeAperiodicPole(pole);

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
% Get T_theta
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function [k ,T_theta] = GetT_theta_old(tf)
    global e_minreal
    [num_, den_] = tfdata(minreal(tf, e_minreal));
    num = fliplr(num_{1});
    k   = num(1);
    T_theta  = num(2)/k; 
end

function [T_theta] = GetT_theta(tf)
    global e_minreal
    tf_ = minreal(tf, e_minreal);
    T_theta = -1 / zero(tf_)
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Check Short Period Design Criteria
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function [wn_design,T_theta_design,dr_design] = GetShortPeriodDesignCriteria()
    %----------------------------------------
    % Get Design Criteria
    %----------------------------------------
    global velocity0 USE_SI_UNITS feet_to_m
    v = velocity0;
    if ~USE_SI_UNITS
        v = v*feet_to_m; 
    end
    
    wn_design      = 0.03*v;
    T_theta_design = 1./(0.75*wn_design);
    dr_design      = 0.5;
end

function CheckShortPeriodDesign(sys,poles,name)

    [wn_design,T_theta_design,dr_design] = GetShortPeriodDesignCriteria;

    %----------------------------------------
    % Analyze System
    %----------------------------------------
    
    [wn,dr,P,T_half] = AnalyzePeriodicPoles(poles);
    T_theta = GetT_theta(sys);
    
    %----------------------------------------
    % Compare
    %----------------------------------------
    
    wn_diff      = wn_design-wn;
    T_theta_diff = T_theta_design-T_theta;
    dr_diff      = dr_design-dr;

    %----------------------------------------
    % Print differences
    %----------------------------------------
    fprintf('wn_org        : %f \n',wn)
    fprintf('T_theta_org   : %f \n',T_theta)
    fprintf('dr_org        : %f \n',dr)
    fprintf('wn_design     : %f \n',wn_design)
    fprintf('T_theta_design: %f \n',T_theta_design)
    fprintf('dr_design     : %f \n',dr_design)
    fprintf('wn_diff       : %f \n',wn_diff  )
    fprintf('T_theta_diff  : %f \n',T_theta_diff  )
    fprintf('dr_diff       : %f \n',dr_diff  )
end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Get Pitch Rate Criteria
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function [CAP,dr,DBqss,qmqs] = GetCAP_and_Gibson(sys,poles)

    T = 0:0.01:7;

    % Calc CAP and Gibson
    global velocity0 USE_SI_UNITS feet_to_m g0
    v = velocity0;
    if ~USE_SI_UNITS
        v = v*feet_to_m; 
    end

    [wn,dr,P,T_half] = AnalyzePeriodicPoles(poles);
    T_theta = GetT_theta(sys);
    CAP = g0 * wn^2 * T_theta / v;
    DBqss = T_theta - 2*dr / wn;
    
    opt = stepDataOptions('StepAmplitude', -1);
    [y,t] = step(sys, T, opt);
    S = stepinfo(y,t);
    qmqs = S.Peak / y(end);
end