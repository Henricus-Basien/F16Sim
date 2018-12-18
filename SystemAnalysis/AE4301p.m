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
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        hold off
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
    
    if PlotQ6
        figure(63);
        grid on
        pzmap(tf_long_Ue_theta)
        ti = title('Longitudinal Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    end

    %.. Phugoid ..
    if PlotQ6
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
    end

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
    
    if PlotQ6
        figure(66);
        grid on
        pzmap(tf_lat_Ua)
        ti = title('Lateral Pole-Zero Map');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
    

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

end

%...........................................................................................................................

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

    Ue__     = 1;
    Xalpha__ = 1;
    Xq__     = 2;
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

    tf_long_Ue_q   = minreal(tf(C_alphaq(Xq__    ,:) * (inv((s*eye(size(A_alphaq,1))-A_alphaq))*B_alphaq(:,Ue__    ))),e_minreal)
    tf_long_Ue_q_4 = minreal(tf(C_long  (Xq_long ,:) * (inv((s*eye(size(A_long  ,1))-A_long  ))*B_long  (:,Ue_long ))),e_minreal)

    T = 0:0.01:7;
    
    if PlotQ7
        figure(74)
        hold on
        %grid on
        [y1,t] = step(tf_long_Ue_q,T);
        plot(t,y1)
        ti = title('2-State Ue-q Step');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        %figure(75)
        %grid on
        [y2,t] = step(tf_long_Ue_q_4,T);
        plot(t,y2)
        xlabel('Time [s]');ylabel('Pitch Rate [deg/s]');
        ti = title('n-State Ue-q Step');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        legend('2-State', '4-State')
        hold off
        
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

    if PlotQ7
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
    
    K = place(A_alphaq, B_alphaq, poles_design);
    kq = K(2);
    kalpha = K(1);
    [y1 t] = step(tf_long_Ue_q);
    [y2 t] = step(tf_long_Ue_q * Compensator_1);
    KComp = y1(end)/y2(end)
    Compensator_1 = Compensator_1 * KComp
    
    %--- Time constant ---
    T_theta = GetT_theta(tf_long_Ue_q*Compensator_1);
    
    Compensator_2 = (1+s*TC_design)/(1+s*T_theta)

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

    if PlotQ7
        figure(712)
        step(tf_long_Ue_q_design)
        ti = title('Short Period - Design');
        xlabel('Time [s]');ylabel('Pitch Rate [deg/s]');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        figure(713)
        pzmap(tf_long_Ue_q_design)
        ti = title('Short Period - Design Pole-Zero Map');
    end

    CheckShortPeriodDesign(tf_long_Ue_q_design,tf_long_Ue_q_design_poles,'Short Period - Design')

    % tf_long = minreal(tf(C_alphaq * (inv((s*eye(size(A_alphaq,1))-A_alphaq))*B_alphaq)),e_minreal)
    % tf_long_design = tf_long * Compensator

    %..............................
    % Check Gains
    %..............................
    
    TC = GetT_theta(tf_long_Ue_q * Compensator_1);

    tf_long_Ue_alpha        = minreal(tf(C_alphaq(Xalpha__,:) * (inv((s*eye(size(A_alphaq,1))-A_alphaq))*B_alphaq(:,Ue__))),e_minreal)
    tf_long_Ue_alpha_design = minreal(tf_long_Ue_alpha * Compensator_1,e_minreal)

    TC = GetT_theta(tf_long_Ue_alpha_design);
    
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
    % Calc CAP and Gibson
    v = velocity0;
    if ~USE_SI_UNITS
        v = v*feet_to_m; 
    end
    
    [wn,dr,P,T_half] = AnalyzePeriodicPoles(tf_long_Ue_q_design_poles);
    [k,TC] = GetTC(tf_long_Ue_q_design);
    CAP = g0 * wn^2 * TC / v;
    DBqss = TC - 2*dr / wn;
    
    opt = stepDataOptions('StepAmplitude', -1);
    [y,t] = step(tf_long_Ue_q_design, T, opt);
    S = stepinfo(y,t);
    qmqs = S.Peak / y(end);
    
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
        loglog(dr, CAP, "x");
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
        loglog(dr, CAP, "x");
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
        loglog(dr, CAP, "x");
        hold off
    end
    
    if PlotQ7 %Plots of Gibson Criteria
       figure(764)
       hold on
       grid on
       title('Gibson Criterion');
       v = [0 1; 0 3; 0.06 3; 0.3 1];
       f = [1 2 3 4];
       patch('Faces', f, 'Vertices', v, 'FaceColor', 'none');
       plot(DBqss, qmqs, "x");
       hold off
    end
    
    if PlotQ7 %Time response of pitch rate and pitch attitude
        figure(765)
        step(tf_long_Ue_q_design, T, opt); %pitch rate
        title('Pitch rate step response');
        figure(766)
        step(tf_long_Ue_q_design*(1/s), T, opt); %pitch attitude
        title('Pitch attitude step response');
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

    x_terrainfollow0 = [dh0,0,0,0,0 , 0,0]
    
    % thrust_terrainfollowing0

    % simOut = sim('TerrainFollowing','SimulationMode','normal','AbsTol','1e-5',...
    %         'SaveState','on','StateSaveName','xout',...
    %         'SaveOutput','on','OutputSaveName','yout',...
    %         'SaveFormat', 'Dataset');
    % outputs = simOut.get('yout')

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
    
    UseOptimalControl = 1;%0;

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
    
    %..............................
    % Run Simulation
    %..............................
    
    sim('TerrainFollowing')
    
    %..............................
    % Evaluate Simulation
    %..............................
    
    t = TerrainFollowing_Output_Altitude.time;
    [altitude,altitude_ref,altitude_ground] = TerrainFollowing_Output_Altitude.signals.values;
    [altitude_err,height_over_ground]       = TerrainFollowing_Output_AltitudeError.signals.values;
    [thrust_inputs]                         = TerrainFollowing_Output_Thrust.signals.values;
    [elevator_inputs]                       = TerrainFollowing_Output_Elevator.signals.values;
    
    % TerrainFollowing_Output_Altitude
    % TerrainFollowing_Output_AltitudeError
    % TerrainFollowing_Output_Thrust
    % TerrainFollowing_Output_Elevator
    
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
                    fprintf('Comstant Reference Reached @ t=%f \n',t(i));
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
        plot(t,altitude       , 'DisplayName','Altitude'        )
        plot(t,altitude_ref   , 'DisplayName','Reference Height')
        plot(t,altitude_ground, 'DisplayName','Ground Height'   )
        legend('Location','southeast')
        ti = title('TerrainFollowing - Flightpath');
        xlabel('Time [s]');
        ylabel('Altitude [m]');
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)
        hold off

        %--- Altitude-Error ---
        figure(82)
        plot(t,height_over_ground, 'DisplayName','Height over Ground')
        yline(d_alt_crit);
        yline(40- c_overshoot_max);
        yline(40);
        yline(40+c_overshoot_max);    
        ti = title('TerrainFollowing - Height over Ground');
        xlabel('Time [s]');
        ylabel('Height [m]');
        ylim([d_alt_crit-1,max(height_over_ground)+1])
        print(gcf, '-dpng', strcat(figpath,'/',ti.String,figext), dpi)

        %--- Inputs ---
        figure(83)
        hold on
        plot(t,thrust_inputs  +thrust0  , 'DisplayName','Thrust'  )
        ylabel('Thrust [' + fu + ']');
        yyaxis right
        plot(t,elevator_inputs+elevator0, 'DisplayName','Elevator')
        ylabel('Elevator [deg]'); ylim([elevator_min*1.05 elevator_max*1.05]);
        legend('Location','southeast')
        ti = title('TerrainFollowing - Control Inputs');
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

function [k ,TC] = GetTC(tf)
    global e_minreal
    [num_, den_] = tfdata(minreal(tf, e_minreal));
    num = fliplr(num_{1});
    k   = num(1);
    TC  = num(2)/k; 
end

function [T_theta] = GetT_theta(tf)
    global e_minreal
    tf_ = minreal(tf, e_minreal);
    T_theta = -1 / zero(tf_)
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
    TC_design = 1./(0.75*wn_design);
    dr_design = 0.5;
end

function CheckShortPeriodDesign(sys,poles,name)

    [wn_design,TC_design,dr_design] = GetShortPeriodDesignCriteria;

    %----------------------------------------
    % Analyze System
    %----------------------------------------
    
    [wn,dr,P,T_half] = AnalyzePeriodicPoles(poles);
    TC = GetT_theta(sys);
    
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
