#ifndef nlplant_H
#define nlplant_H

void atmos(double,double,double*);          /* Used by both */
void accels(double*,double*,double*);       /* Used by both */
void nlplant(double *xu, double *xdot);

#endif//nlplant_H