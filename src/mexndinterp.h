#ifndef mexndinterp_h
#define mexndinterp_h

typedef struct{
	int nDimension;/* Number of dimensions*/
	int *nPoints;   /* number of points along each direction */
}ND_INFO;

/*******************************************/
/*    Creation of integer vector 	   */
/*******************************************/
int *intVector(int n);
/*********************************************/
/* 	Create a double Vector		     */
/*********************************************/
double *doubleVector(int n);
/*******************************************/
/*    Creation of integer MATRIX 	   */
/*******************************************/
int **intMatrix(int n,int m);
/*********************************************/
/* 	Create a double MATRIX		     */
/*********************************************/
double **doubleMatrix(int n,int m);
/*********************************************/
/*  	Free integer matrix			  */
/*********************************************/
void freeIntMat(int **mat,int n,int m);
/*********************************************/
/*   	Free double matrix			  */
/*********************************************/
void freeDoubleMat(double **mat,int n,int m);
/*********************************************/
/*    Print out error and exit program       */
/*********************************************/
void ErrMsg(char *m);
/************************************************/
/*    Get the indices of the hyper cube in the  */
/*    grid in which the point lies              */
/************************************************/
int **getHyperCube(double **X,double *V,ND_INFO ndinfo);
/*********************************************************************************
 indexVector contains the co-ordinate of a point in the ndimensional grid
 the indices along each axis are assumed to begin from zero
 *********************************************************************************/
int getLinIndex(int *indexVector,ND_INFO ndinfo);
double linearInterpolate(double *T,double *V,double **X,ND_INFO ndinfo);
double interpn(double **X,double *Y,double *x,ND_INFO ndinfo);

#endif // mexndinterp_h