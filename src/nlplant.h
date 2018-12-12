#ifndef nlplant_H
#define nlplant_H

void atmos(double,double,double*);          /* Used by both */
void accels(double*,double*,double*);       /* Used by both */
void nlplant_(double *xu, double *xdot);

#endif//nlplant_H