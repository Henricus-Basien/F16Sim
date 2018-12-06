#ifndef hifi_F16_AeroData_c_H
#define hifi_F16_AeroData_c_H

void hifi_C(            double alpha, double beta, double el, double *retVal); //{}
void hifi_damping(      double alpha,                         double *retVal); //{}
void hifi_C_lef(        double alpha, double beta,            double *retVal); //{}
void hifi_damping_lef(  double alpha,                         double *retVal); //{}
void hifi_rudder(       double alpha, double beta,            double *retVal); //{}
void hifi_ailerons(     double alpha, double beta,            double *retVal); //{}
void hifi_other_coeffs( double alpha,              double el, double *retVal); //{}  

#endif // hifi_F16_AeroData_c_H