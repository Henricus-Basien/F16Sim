F16 Simulator - Standalone C executable + Live Piloted Python Simulation + Matlab Controller Analysis
Based on the 'Non-linear F-16 Simulation using Simulink and Matlab' of the University of Minnesota. (Converted into SI units)

The Live Python Simulation can be found in the folder 'LiveSim' and exectuted by calling 'python F16Sim.py'
Extended model analysis can be found in the folder 'SystemAnalysis', using Matlab to analyze the system and ultimatly design an optimal terrainfollowing controller.

To compile the exectutable for Windows or Linux just run 'make' in the root folder!
To mex the model for Matlab, run the content of the 'mexfile' from within the 'src' directory in Matlab.