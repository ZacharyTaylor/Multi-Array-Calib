/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using cProb(err, VA, VB, rot)
 */ 

#include "mex.h"
#include "matrix.h"
#define _USE_MATH_DEFINES
#include <cmath>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 3 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 3 inputs and 1 output.");
    }
        
    //gets size of variables
    const int lenA = mxGetDimensions(prhs[0])[1];
    const int lenB = mxGetDimensions(prhs[1])[1];
    
    //get value of input variables
    float* AIn = (float*) mxGetData(prhs[0]);
    float* BIn = (float*) mxGetData(prhs[1]);
    float* F = (float*) mxGetData(prhs[2]);
    
    //setup outputs
    plhs[0] = mxCreateNumericMatrix(lenA, 1, mxSINGLE_CLASS, mxREAL);
    float* out = (float*) mxGetData(plhs[0]);
        
    for(int a = 0; a < lenA; a++){
        float err = 100000;
        float* A = &AIn[2*a];
        float* B;
        for(int b = 0; b < lenB; b++){
    
            B = &BIn[2*b];
        
            float temp = B[0]*(A[0]*F[0] + A[1]*F[1] + F[2]) + B[1]*(A[0]*F[3] + A[1]*F[4] + F[5]) + A[0]*F[6] + A[1]*F[7] + F[8];
            //temp *= sqrt((A[0] - B[0])*(A[0] - B[0]) + (A[1] - B[1])*(A[1] - B[1]));
            if(abs(temp) < err){
                err = abs(temp);
                out[a] = b;
            }
        }
        B = &BIn[2*(int)(out[a])];
        out[a] = B[0]*(A[0]*F[0] + A[1]*F[1] + F[2]) + B[1]*(A[0]*F[3] + A[1]*F[4] + F[5]) + A[0]*F[6] + A[1]*F[7] + F[8];
        out[a] = abs(out[a]);
    }
}
