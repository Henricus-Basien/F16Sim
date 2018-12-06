#ifndef lofi_F16_AeroData_c_H
#define lofi_F16_AeroData_c_H

void damping(  double alpha,                           double *coeff);
void dmomdcon( double alpha, double beta,              double *coeff);
void clcn(     double alpha, double beta,              double *coeff);
void cxcm(     double alpha, double dele,              double *coeff);
void cz(       double alpha, double beta, double dele, double *coeff);

#endif // lofi_F16_AeroData_c_H