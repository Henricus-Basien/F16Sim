/*
Created on Thursday 06.12.2018
Copyright (©) Henricus N. Basien
Author: Henricus N. Basien
Email: Henricus@Basien.de
*/

//****************************************************************************************************
// Includes
//****************************************************************************************************

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// External
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//----------------------------------------
// Boolean
//----------------------------------------


typedef int bool_;
#define true 1
#define false 0

//----------------------------------------
// Timing
//----------------------------------------

//#define _POSIX_C_SOURCE 199309L
#include <time.h>
#include <sys\time.h>

#define s_to_ms 1000.0
#define s_to_us 1000000.0
#define s_to_ns 1000000000.0

time_t getTime_s(){
	struct timeval ts; //struct timespec ts;
	gettimeofday(&ts , NULL);//timespec_get(&ts, TIME_UTC);
	return ts.tv_sec;
}

time_t getTime_us(){
	struct timeval ts; //struct timespec ts;
	gettimeofday(&ts , NULL);//timespec_get(&ts, TIME_UTC);
	return ts.tv_usec;
}

// time_t getTime_ns(){
// 	struct timeval ts; //struct timespec ts;
// 	gettimeofday(&ts , NULL);//timespec_get(&ts, TIME_UTC);
// 	return ts.tv_nsec;
// }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Internal
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "settings.h"
#include "nlplant.h"

//****************************************************************************************************
// Main
//****************************************************************************************************

//#define TrackTimeInSingleRun

bool_ Print = false;//true;
#ifdef USE_SI_UNITS
	bool_ Convert = true;
#else
	bool_ Convert = false;
#endif
char *OutputFile = "F16Sim_output.csv";

//================================================================================
// Set State
//================================================================================

#define NrStates 18
double xu    [NrStates] = {0.0};
double xdot  [NrStates] = {0.0};

char   xnames[NrStates][100];
char   xunits[NrStates][10];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Conversions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define feet_to_m    0.3048
#define lbf_to_N     4.448222
#define lb_ft2_to_Pa 47.880258888889

double xu_IU_to_SI[NrStates]   = {1.0};
double xdot_IU_to_SI[NrStates] = {1.0};

//----------------------------------------
// xu
//----------------------------------------

void Init_xu_IU_to_SI(){

	//--- Init ---
	for (int i=0;i<NrStates;i++){
		xu_IU_to_SI[i] = 1.0;
	}

	//--- Position ---
	for (int i=0;i<3;i++){
		xu_IU_to_SI[i] = feet_to_m;
	}

	//--- Velocity ---
	xu_IU_to_SI[6] = feet_to_m;
	//--- Thrust ---
	xu_IU_to_SI[12] = lbf_to_N;

}

void Convert_xu_IU_to_SI(){
	for (int i=0;i<NrStates;i++){
		xu[i]*=xu_IU_to_SI[i];
	}
}

void Convert_xu_SI_to_IU(){
	for (int i=0;i<NrStates;i++){
		xu[i]/=xu_IU_to_SI[i];
	}
}

//----------------------------------------
// xdot
//----------------------------------------

void Init_xdot_IU_to_SI(){

	//--- Init ---
	for (int i=0;i<NrStates;i++){
		xdot_IU_to_SI[i] = 1.0;
	}

	//--- Copy from State Vector ---
	for (int i=0;i<NrStates;i++){
		xdot_IU_to_SI[i] = xu_IU_to_SI[i];
	}
	//--- Accelerations/Load Factors ---
	for (int i=12;i<15;i++){
		xu_IU_to_SI[i] = 9.81;//1; // [g/g]
	}
	//--- Dynamic Pressure ---
	xu_IU_to_SI[16] = lb_ft2_to_Pa;
	//--- Static Pressure ---
	xu_IU_to_SI[17] = lb_ft2_to_Pa;
}	

void Convert_xdot_IU_to_SI(){
	for (int i=0;i<NrStates;i++){
		xdot[i]*=xdot_IU_to_SI[i];
	}
}

void Convert_xdot_SI_to_IU(){
	for (int i=0;i<NrStates;i++){
		xdot[i]/=xdot_IU_to_SI[i];
	}
}

//----------------------------------------
// Combined
//----------------------------------------

void Convert_IU_to_SI(){
	Convert_xu_IU_to_SI();
	Convert_xdot_IU_to_SI();
}

void Convert_SI_to_IU(){
	Convert_xu_SI_to_IU();
	Convert_xdot_SI_to_IU();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// State - xu
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Init_xu(){
	//----------------------------------------
	// Position & Orientation
	//----------------------------------------

	xu[0]  = 0.0; strcpy(xnames[0],"Postion North"); strcpy(xunits[0],"m");//"ft" ); /* north position */
	xu[1]  = 0.0; strcpy(xnames[1],"Postion East" ); strcpy(xunits[1],"m");//"ft" ); /* east position */ 
	xu[2]  = 0.0; strcpy(xnames[2],"Altitude"     ); strcpy(xunits[2],"m");//"ft" ); /* altitude */      
	xu[3]  = 0.0; strcpy(xnames[3],"Roll  Angle"  ); strcpy(xunits[3],"rad"); /* phi - orientation angles in rad. */
	xu[4]  = 0.0; strcpy(xnames[4],"Pitch Angle"  ); strcpy(xunits[4],"rad"); /* theta - orientation angles in rad. */
	xu[5]  = 0.0; strcpy(xnames[5],"Yaw   Angle"  ); strcpy(xunits[5],"rad"); /* psi - orientation angles in rad. */

	//----------------------------------------
	// Rates & Direction
	//----------------------------------------

	xu[6]  = 100.0; strcpy(xnames[6] ,"Velocity"         ); strcpy(xunits[6] ,"m/s");//"ft/s");  /* total velocity */
	xu[7]  = 0.0  ; strcpy(xnames[7] ,"Angle of Attack"  ); strcpy(xunits[7] ,"rad" );  /* angle of attack in radians */
	xu[8]  = 0.0  ; strcpy(xnames[8] ,"Angle of Sideslip"); strcpy(xunits[8] ,"rad" );  /* sideslip angle in radians */
	xu[9]  = 0.0  ; strcpy(xnames[9] ,"Roll  Rate"       ); strcpy(xunits[9] ,"rad/s"); /* Roll Rate --- rolling  moment is Lbar */
	xu[10] = 0.0  ; strcpy(xnames[10],"Pitch Rate"       ); strcpy(xunits[10],"rad/s"); /* Pitch Rate--- pitching moment is M */
	xu[11] = 0.0  ; strcpy(xnames[11],"Yaw   Rate"       ); strcpy(xunits[11],"rad/s"); /* Yaw Rate  --- yawing   moment is N */

	//----------------------------------------
	// Control inputs
	//----------------------------------------

	xu[12] = 5000.0*lbf_to_N; strcpy(xnames[12] ,"Thrust            [1000 ,19000]"); strcpy(xunits[12],"N");//"lbf"); /* thrust                        | [1000 ,19000] */
	xu[13] = 0.0            ; strcpy(xnames[13] ,"Elevator          [-25  ,+25  ]"); strcpy(xunits[13],"°");   /* Elevator setting in degrees.         | [-25  ,+25  ] */
	xu[14] = 0.0            ; strcpy(xnames[14] ,"Ailerons          [-21.5,+21.5]"); strcpy(xunits[14],"°");   /* Ailerons mex setting in degrees.     | [-21.5,+21.5] */
	xu[15] = 0.0            ; strcpy(xnames[15] ,"Rudder            [-30  ,+30  ]"); strcpy(xunits[15],"°");   /* Rudder setting in degrees.           | [-30  ,+30  ] */
	xu[16] = 0.0            ; strcpy(xnames[16] ,"Leading edge Flap [-25  ,+0   ]"); strcpy(xunits[16],"°");   /* Leading edge flap setting in degrees | [-25  ,+0   ] */

	xu[17] = 1.0; strcpy(xnames[17] ,"Fidelity Setting [0=LOW,1=HIGH]"); strcpy(xunits[17],"N/A"); // 1.0;   /* fi_flag */
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Init
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitState(){
	Init_xu();
	Init_xu_IU_to_SI();
	Init_xdot_IU_to_SI();
}

//================================================================================
// Output
//================================================================================

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Print
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PrintState(){
	printf("****************************************************\n");
	for (int i=0;i<NrStates;i++){
		printf("State (+Der) # %d '%s [%s]' \t %f \t %f \n",i,xnames[i],xunits[i],xu[i],xdot[i]);
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Export
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ExportState(){
	FILE *f = fopen(OutputFile, "w");
	if (f == NULL){
	    printf("Error opening file!\n");
	    exit(1);
	}

	//--- State ---
	if (0){
		for (int i=0;i<NrStates;i++){
			fprintf(f,"%f,",xu[i]);
			//fprintf(f,"%f , \t",xu[i]);
		}
		fprintf(f,"\n");
	}
	//--- Derivatives ---
	for (int i=0;i<NrStates;i++){
		fprintf(f,"%f,",xdot[i]);
		//fprintf(f,"%f , \t",xdot[i]);
	}
	fprintf(f,"\n");
	
	fclose(f);
}

void Export_plus(){
	//--- Convert ---
	if (Convert) Convert_IU_to_SI();
	//--- Print ---
	if (Print) PrintState();
	//--- Export ---
	ExportState();
}

//================================================================================
// Update Simulation
//================================================================================

bool_ PrintData  = false;//true;//false;
bool_ ExportData = false;//true;//false;

void UpdateSimulation(double *xu, double *xdot){
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Run Simulation
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	nlplant_(xu, xdot);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Print Output
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	if (PrintData) PrintState();

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Export Output
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	if (ExportData) ExportState();

}

void UpdateSimulation_plus(double *xu, double *xdot){
	//--- Convert ---
	if (Convert) Convert_SI_to_IU();
	//--- Update ---	
	UpdateSimulation(xu,xdot);
}

//================================================================================
// Run Simulation
//================================================================================

void RunSimulation(){

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Init
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	int t_max = 3600; // [s]

	time_t t_us  = getTime_us(); // [us]
	time_t dt_us = 0;            // [us]
	double t0  = getTime_s()/1.0; // [s]
	double t  = 0.0; // [s]
	double dt = 0.0; // [s]
	double Hz = 0.0; // [Hz]

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Run
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//for (unsigned long RunNr=0;RunNr<MaxNrRuns;RunNr++){
	unsigned long RunNr = 0;
	while (true){
		RunNr+=1;
		if (t-t0>t_max) break;
		//----------------------------------------
		// Run Simulation
		//----------------------------------------
		UpdateSimulation_plus(xu,xdot);
		//----------------------------------------
		// Increment Time
		//----------------------------------------
		dt_us = getTime_us()-t_us;
		t_us  = getTime_us();
		dt = dt_us/s_to_us;
		// t  = t_us /s_to_us;
		t  = getTime_s()/1.0;
		if (dt !=0){
			Hz = 1./dt;
		}
		printf("# t/dt/FPS [s/us/Hz]: %lu \t: %f \t/ %f \t/ %f \n",RunNr,t-t0,dt*s_to_us,Hz);
		//----------------------------------------
		// Increment/Integrate State
		//----------------------------------------
		for (int i=0;i<NrStates;i++){
			xu[i]+=xdot[i]*dt;
		}

		Export_plus();
	}
}

//================================================================================
// Main
//================================================================================

int main(int argc, char **argv){

	#ifdef TrackTimeInSingleRun
		unsigned long t_us = getTime_us();
		float dt_Sim;
		float Hz_Sim;
	#endif

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Init State
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	InitState();

	//--- Convert CLI parameters ---
	if (argc-1>NrStates){
		printf("ERROR: NrInputs > NrStates |  %d > %d\n",argc-1,NrStates);
	}
	for (int i=0;i<argc-1;i++){
		double temp = strtod(argv[i+1],NULL);
		xu[i] = temp;
		if (Print) printf("Set Initial Value '%s [%s]' to %f\n",xnames[i],xunits[i],xu[i]);
	}

	#ifdef TrackTimeInSingleRun
		unsigned long dt_us_init = getTime_us()-t_us;
		float dt_init = dt_us_init/s_to_us;
		float Hz_init = 1./dt_init;
		printf("F16Sim-Init  :: dt/FPS [ms/Hz]: %f \t/ %f \n",dt_init*s_to_ms,Hz_init);
	#endif

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Simulate Continously
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	if (0){
		RunSimulation();
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Update Once
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	else{
	
		//----------------------------------------
		// Update
		//----------------------------------------
		
		UpdateSimulation_plus(xu,xdot);

		#ifdef TrackTimeInSingleRun
			unsigned long dt_us_Sim = getTime_us()-t_us;
			dt_Sim = dt_us_Sim/s_to_us;
			Hz_Sim = 1./dt_Sim;
			printf("F16Sim-Sim   :: dt/FPS [ms/Hz]: %f \t/ %f \n",dt_Sim          *s_to_ms,Hz_Sim);
		#endif

		//----------------------------------------
		// Export
		//----------------------------------------
		
		Export_plus();
		
	}

	#ifdef TrackTimeInSingleRun
		unsigned long dt_us = getTime_us()-t_us;
		float dt = dt_us/s_to_us;
		float Hz = 1./dt;
		printf("F16Sim-Export:: dt/FPS [ms/Hz]: %f \t/ %f \n",(dt-dt_Sim)*s_to_ms,1./(dt-dt_Sim));
		printf("F16Sim-Total :: dt/FPS [ms/Hz]: %f \t/ %f \n",dt          *s_to_ms,Hz);
	#endif

	return 0;
}

//****************************************************************************************************
// Matlab Mex
//****************************************************************************************************

#ifdef USE_SI_UNITS
	void nlplant(double *xu, double *xdot){
		#pragma message "INFO: Using SI Units!"
		UpdateSimulation_plus(xu,xdot);
	}
#else
	void nlplant(double *xu, double *xdot){
		#pragma message "INFO: Using Imperial Units!"
		nlplant_(             xu,xdot);
	}
#endif

#ifdef COMPILE_TO_MEX

	#pragma message "WARNING: Compile to MEX (Matlab) is activated! For 'conventional' build, please deactivate the COMPILE_TO_MEX proprocessor macro in the 'settings.h' file!"

	#include "mex.h"
	/*########################################*/
	/*### Added for mex function in matlab ###*/
	/*########################################*/

	int fix(double);
	int sign(double);

	void mexFunction(int nlhs, mxArray *plhs[],
	               int nrhs, const mxArray *prhs[])
	{

	#define XU prhs[0]
	#define XDOTY plhs[0]

	int i;
	double *xup, *xdotp;

	if (mxGetM(XU)==18 && mxGetN(XU)==1){ 

	    /* Calling Program */
	    xup = mxGetPr(XU);
	    XDOTY = mxCreateDoubleMatrix(18, 1, mxREAL);
	    xdotp = mxGetPr(XDOTY);

	    nlplant(xup,xdotp);

	    /* debug
	    for (i=0;i<=14;i++){
	      printf("xdotp(%d) = %e\n",i+1,xdotp[i]);
	    }
	    end debug */

	} /* End if */
	else{ 
	    mexErrMsgTxt("Input and/or output is wrong size.");
	} /* End else */

	} /* end mexFunction */

	/*########################################*/
	/*########################################*/
#endif