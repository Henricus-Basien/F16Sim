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

typedef int bool;
#define true 1
#define false 0

//----------------------------------------
// Timing
//----------------------------------------

#include <time.h>

#define s_to_ms 1000.0
#define s_to_us 1000000.0
#define s_to_ns 1000000000.0

time_t getTime_s(){
	struct timespec ts;
	timespec_get(&ts, TIME_UTC);
	return ts.tv_sec;
}

time_t getTime_ns(){
	struct timespec ts;
	timespec_get(&ts, TIME_UTC);
	return ts.tv_nsec;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Internal
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "nlplant.h"

//****************************************************************************************************
// Main
//****************************************************************************************************

bool Convert = true;

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
	FILE *f = fopen("output.csv", "w");
	if (f == NULL){
	    printf("Error opening file!\n");
	    exit(1);
	}

	for (int i=0;i<NrStates;i++){
		fprintf(f,"%f , \t",xu[i]);
	}
	fprintf(f,"\n");
	for (int i=0;i<NrStates;i++){
		fprintf(f,"%f , \t",xdot[i]);
	}
	fprintf(f,"\n");
	
	fclose(f);
}

//================================================================================
// Update Simulation
//================================================================================

bool PrintData  = false;//true;//false;
bool ExportData = false;//true;//false;

void UpdateSimulation(double *xu, double *xdot){
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Run Simulation
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	nlplant(xu, xdot);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Print Output
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	if (PrintData) PrintState();

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Export Output
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	if (ExportData) ExportState();

}

//================================================================================
// Run Simulation
//================================================================================

void RunSimulation(){

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Init
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	int MaxNrRuns = 10000;

	time_t t_ns  = getTime_ns(); // [ns]
	time_t dt_ns = 0;            // [ns]
	double t  = 0.0; // [s]
	double dt = 0.0; // [s]
	double Hz = 0.0; // [Hz]

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Run
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	for (unsigned long RunNr=0;RunNr<MaxNrRuns;RunNr++){
		//----------------------------------------
		// Run Simulation
		//----------------------------------------
		UpdateSimulation(xu,xdot);
		//----------------------------------------
		// Increment Time
		//----------------------------------------
		dt_ns = getTime_ns()-t_ns;
		t_ns  = getTime_ns();
		dt = dt_ns/s_to_ns;
		// t  = t_ns /s_to_ns;
		t  = getTime_s()/1.0;
		if (dt !=0){
			Hz = 1./dt;
		}
		printf("# t/dt/FPS [s/us/Hz]: %lu \t: %f \t/ %f \t/ %f \n",RunNr,t,dt*s_to_us,Hz);
		//----------------------------------------
		// Increment/Integrate State
		//----------------------------------------
		for (int i=0;i<NrStates;i++){
			xu[i]+=xdot[i]*dt;
		}
	}
}

//================================================================================
// Main
//================================================================================

bool Print   = true;

int main(int argc, char **argv){

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
		printf("Set Initial Value '%s [%s]' to %f\n",xnames[i],xunits[i],xu[i]);
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Update
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//--- Convert ---
	if (Convert) Convert_SI_to_IU();
	//--- Update ---	
	UpdateSimulation(xu,xdot);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Export
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	//--- Convert ---
	if (Convert) Convert_IU_to_SI();
	//--- Print ---
	if (Print) PrintState();
	//--- Export ---
	ExportState();

	return 0;
}
