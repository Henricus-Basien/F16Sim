Settings

%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
%================================================
clear;

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

global USE_SI_UNITS lbf_to_N lu
global altitude0 velocity0
global PlotPoles PlotBode

newline = sprintf('\n');

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Trim
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

%% Trim aircraft to desired altitude and velocity
%%

if exist("altitude0") == 0 || isempty(altitude0)
    altitude = input('Enter the altitude for the simulation ('+lu+')  :  ');
else
    altitude = altitude0;
end
if exist("velocity0") == 0 || isempty(velocity0)
    velocity = input('Enter the velocity for the simulation ('+lu+'/s)  :  ');
else
    velocity = velocity0;
end

%% Initial guess for trim
%%
if (USE_SI_UNITS == 1)
    thrust = 5000*lbf_to_N;          % thrust, lbs
    elevator = -0.09;       % elevator, degrees
    alpha = 8.49;              % AOA, degrees
    rudder = -0.01;             % rudder angle, degrees
    aileron = 0.01;            % aileron, degrees
else
    thrust = 5000;          % thrust, lbs
    elevator = -0.09;       % elevator, degrees
    alpha = 8.49;              % AOA, degrees
    rudder = -0.01;             % rudder angle, degrees
    aileron = 0.01;            % aileron, degrees
end

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 1;
[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
[A_hi,B_hi,C_hi,D_hi] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3); ...
        dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
        dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Make state space model
%%
SS_hi = ss(A_hi,B_hi,C_hi,D_hi);
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);


%% Make MATLAB matrix
%%
mat_hi = [A_hi B_hi; C_hi D_hi];
mat_lo = [A_lo B_lo; C_lo D_lo];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the longitude A matrix
%%
A_longitude_hi = mat_hi([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude B matrix
%%
B_longitude_hi = mat_hi([3 5 7 8 11 13 14], [19 20]);
B_longitude_lo = mat_lo([3 5 7 8 11 13 14], [19 20]);

%% Select the components that make up the longitude C matrix
%%
C_longitude_hi = mat_hi([21 23 25 26 29], [3 5 7 8 11 13 14]);
C_longitude_lo = mat_lo([21 23 25 26 29], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude D matrix
%%
D_longitude_hi = mat_hi([21 23 25 26 29], [19 20]);
D_longitude_lo = mat_lo([21 23 25 26 29], [19 20]);

SS_long_hi = ss(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
SS_long_lo = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Directional %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the lateral A matrix
%%
A_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);
A_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral B matrix
%%
B_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [19 21 22]);
B_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [19 21 22]);

%% Select the components that make up the lateral C matrix
%%
C_lateral_hi = mat_hi([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);
C_lateral_lo = mat_lo([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral D matrix
%%
D_lateral_hi = mat_hi([22 24 25 27 28 30], [19 21 22]);
D_lateral_lo = mat_lo([22 24 25 27 28 30], [19 21 22]);

SS_lat_hi = ss(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);
SS_lat_lo = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

%% Make longitudal direction SYSTEM matrix
%%
sys_long_hi = pck(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
sys_long_lo = pck(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%% Make lateral direction SYSTEM matrix and Find poles for hifi
%%
sys_lat_hi = pck(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);

long_poles_hi = spoles(sys_long_hi);
lat_poles_hi = spoles(sys_lat_hi);



%% Make lateral direction SYSTEM matrix and Find poles for lofi
%%
sys_lat_lo = pck(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

long_poles_lo = spoles(sys_long_lo);
lat_poles_lo = spoles(sys_lat_lo);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Display results


clc;

disp(sprintf('Altitude: %.3f ft.', altitude));
disp(sprintf('Velocity: %.3f ft/s\n\n', velocity));

disp('For HIFI Model:  ');
disp('Longitudal Direction:  ');
disp(newline);

disp('A =')
for i=1:length( A_longitude_hi(:,1) )
    mprintf([ A_longitude_hi(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_longitude_hi(:,1) )
    mprintf([ B_longitude_hi(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_longitude_hi(:,1) )
    mprintf([ C_longitude_hi(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_longitude_hi(:,1) )
    mprintf([ D_longitude_hi(i,:) ],'  %.3e ')
end %for

rifd(long_poles_hi)

disp(newline);

disp('Lateral Direaction:  ');

disp(newline);

disp('A =')
for i=1:length( A_lateral_hi(:,1) )
    mprintf([ A_lateral_hi(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_lateral_hi(:,1) )
    mprintf([ B_lateral_hi(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_lateral_hi(:,1) )
    mprintf([ C_lateral_hi(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_lateral_hi(:,1) )
    mprintf([ D_lateral_hi(i,:) ],'  %.3e ')
end %for

rifd(lat_poles_hi)

disp(newline);
disp(newline);
disp('For LOFI Model:  ');
disp('Longitudal Direction:  ');
disp(newline);

disp('A =')
for i=1:length( A_longitude_lo(:,1) )
    mprintf([ A_longitude_lo(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_longitude_lo(:,1) )
    mprintf([ B_longitude_lo(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_longitude_lo(:,1) )
    mprintf([ C_longitude_lo(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_longitude_lo(:,1) )
    mprintf([ D_longitude_lo(i,:) ],'  %.3e ')
end %for

% Display the real, imaginary, frequency (magnitude) and damping ratios
rifd(long_poles_lo)

disp(newline);

disp('Lateral Direaction:  ');

disp(newline);

disp('A =')
for i=1:length( A_lateral_lo(:,1) )
    mprintf([ A_lateral_lo(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_lateral_lo(:,1) )
    mprintf([ B_lateral_lo(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_lateral_lo(:,1) )
    mprintf([ C_lateral_lo(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_lateral_lo(:,1) )
    mprintf([ D_lateral_lo(i,:) ],'  %.3e ')
end %for

%================================================================================
% Laplace Pole-Zero Plots
%================================================================================

if exist("PlotPoles") == 0 || isempty(PlotPoles)
    flag_poles = input('Do you want to display the Pole-Zero plots for the system (y/n):  ','s'); 
else
    flag_poles = PlotPoles;
end

if flag_poles == "y"

    % Display the real, imaginary, frequency (magnitude) and damping ratios
    rifd(lat_poles_lo)

    %% All Poles
    figure(1); 
    pzmap(SS_hi, 'r', SS_lo, 'b');
    title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lofi Red = hifi.', altitude, velocity);
    title(title_string);
    sgrid;

    %% Long. Poles
    %%
    figure(2); 
    pzmap(SS_long_hi, 'r', SS_long_lo, 'b');
    title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
    title(title_string);
    sgrid;

    %% Lat. Poles
    %%
    figure(3); 
    pzmap(SS_lat_hi, 'r', SS_lat_lo, 'b');
    title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLateral Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
    title(title_string);
    sgrid;
else
    disp("Pole-Zero Plots are not being plotted...")
end

%================================================================================
% Bode Plots
%================================================================================

% Create Bode Plots

if exist("PlotBode") == 0 || isempty(PlotBode)
    flag_bode = input('Do you want to display the Bode plots for the system (y/n):  ','s'); 
else
    flag_bode = PlotBode;
end

if flag_bode == "y"

    omega = logspace(-2,2,100);

    sysg_lat_hi = frsp(sys_lat_hi,omega);
    sysg_lat_lo = frsp(sys_lat_lo,omega);

    sysg_long_hi = frsp(sys_long_hi,omega);
    sysg_long_lo = frsp(sys_long_lo,omega);

    figure;
    BodeCount = 0;
    for state = 1:1:5
        for control = 1:1:2
            BodeCount = BodeCount +1;
            title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
            vplot('bode', sel(sysg_long_hi,state,control), 'b--', sel(sysg_long_lo,state,control), 'r');
            disp(title_string);
            legend('hifi', 'lofi');
            pause;
        end
    end

    for state = 1:1:6
        for control = 1:1:3
            BodeCount = BodeCount + 1;
            title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
            vplot('bode', sel(sysg_lat_hi,state,control), 'b--', sel(sysg_lat_lo,state,control), 'r');
            disp(title_string);
            legend('hifi', 'lofi');
            pause;
        end
    end
else
    disp("Bode Plots are not being plotted...")
end
