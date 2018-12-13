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

fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')
fprintf('                          Q5                                \n')
fprintf('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n')

fprintf('----------------------------------------\n')
fprintf('                  Q5.2                  \n')
fprintf('----------------------------------------\n')

fprintf('Trimming Linearized Model\n')
%FindF16Dynamics

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
fprintf('Elements Nz depends on: ')
for i = Nz_index
	fprintf('%s, ',StateNames(i,:));
end
fprintf('\n');

fprintf('----------------------------------------\n')
fprintf('                  Q5.5                  \n')
fprintf('----------------------------------------\n')

tf_Ue_Nz = minreal(tf(C_lo(YNz,:) * (inv((s*eye(18)-A_lo))*B_lo(:,Ue))),e_minreal)

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
lines = [];
opt = stepDataOptions('StepAmplitude', -1);
T = 0:0.01:6;   
for xa__ = xa_
    hold on
    xa = xa__
	fprintf('Analyzing xa: %f \n',xa)
	FindF16Dynamics
	if ApplyStateSpaceSimplification == 1
		SimplifyStatespace
	end
	tf_Ue_Nz_ = minreal(tf(C_lo(YNz,:) * (inv((s*eye(18)-A_lo))*B_lo(:,Ue))),e_minreal)
    [y,t] = step(tf_Ue_Nz, T, opt);
    plot(t,y);
	%PlotElevatorStepInput(tf_Ue_Nz_)

end
grid on
title('Negative Elevator step input');
xlabel('Time [s]');
ylabel('Normal acceleration in z [g]');
hold off
    

%================================================================================
% Addendum
%================================================================================

input('Press ENTER to close the Analysis...');
close all

%****************************************************************************************************
% Functions
%****************************************************************************************************

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Plot Elevator Step Input
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


function PlotElevatorStepInput(tf_)
	opt = stepDataOptions('StepAmplitude', -1);
	T = 0:0.01:6;   
	grid on
	[y,t] = step(tf_, T, opt);
    plot(t,y);
	title('Negative Elevator step input');
	xlabel('Time [s]');
	ylabel('Normal acceleration in z [g]');
end

