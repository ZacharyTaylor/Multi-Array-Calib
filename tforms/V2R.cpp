/*matlab was bottleneck on V2R for some weird reason so heres it in c*/
#include "mex.h"
#include "matrix.h"
#define _USE_MATH_DEFINES
#include <cmath>

void V2R(const double* const vect, double* const R){
    
    double s,c,k,x,y,z;
    
    //get rotation mag
    double m = sqrt(vect[0]*vect[0] + vect[1]*vect[1] + vect[2]*vect[2]);
    
    //check for zero case and set angles
    if( m < 1.0e-12){
        s = 0;
        c = 1;
        k = 0;
        x = 0;
        y = 0;
        z = 0;
    }
    else{
        s = sin(m);
        c = cos(m);
        k = 1 - c;
        x = vect[0]/m;
        y = vect[1]/m;
        z = vect[2]/m;
    }
	
    //get rotation matrix
     R[0] = k*x*x+c;
     R[1] = k*x*y + s*z;
     R[2] = k*x*z - s*y;
     R[3] = k*x*y - s*z;
     R[4] = k*y*y + c;
     R[5] = k*y*z + s*x;
     R[6] = k*x*z + s*y;
     R[7] = k*y*z - s*x;
     R[8] = k*z*z + c;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if(nrhs != 1 || (nlhs != 1 && nlhs != 0))
        mexErrMsgIdAndTxt("V2R:BadNArgs", "Need 1 inputs and 1 output.");
    
    //check input type
    if(!mxIsDouble(prhs[0]))
        mexErrMsgIdAndTxt("V2R:notDouble", "Input must be of type double");
        
    //check input size
    if((mxGetM(prhs[0])!=1) || (mxGetN(prhs[0])!=3) || (mxGetNumberOfElements(prhs[0]) != 3))
        mexErrMsgIdAndTxt("V2R:worngSize", "Input must be a 1x3 vector");
           
    //get value of input variable
    double * V = mxGetPr(prhs[0]);
    
    //setup output
    plhs[0] = mxCreateNumericMatrix(3, 3, mxDOUBLE_CLASS, mxREAL);
    double *R = mxGetPr(plhs[0]);
    
    //convert vector to rotation matrix
    double s,c,k,x,y,z;
    
    //get rotation mag
    double m = sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
    
    //check for zero case and set angles
    if( m < 1.0e-12){
        s = 0;
        c = 1;
        k = 0;
        x = 0;
        y = 0;
        z = 0;
    }
    else{
        s = sin(m);
        c = cos(m);
        k = 1 - c;
        x = V[0]/m;
        y = V[1]/m;
        z = V[2]/m;
    }
	
    //get rotation matrix
     R[0] = k*x*x+c;
     R[1] = k*x*y + s*z;
     R[2] = k*x*z - s*y;
     R[3] = k*x*y - s*z;
     R[4] = k*y*y + c;
     R[5] = k*y*z + s*x;
     R[6] = k*x*z + s*y;
     R[7] = k*y*z - s*x;
     R[8] = k*z*z + c;
}
