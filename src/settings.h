#ifndef settings_H
#define settings_H

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// System
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define USE_SI_UNITS

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MEX
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if __has_include("mex.h") //#ifdef MX_API_VER
	#define COMPILE_TO_MEX
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Data Path
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef COMPILE_TO_MEX
	#define DATA_PATH "./../data/"
#else
	#define DATA_PATH "./data/"
#endif

#endif//settings_H