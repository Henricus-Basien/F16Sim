#ifndef settings_H
#define settings_H

#define USE_SI_UNITS

#if __has_include("mex.h") //#ifdef MX_API_VER
	#define COMPILE_TO_MEX
#endif

#endif//settings_H