/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using cProb(V, rot)
 */ 

#include "mex.h"
#include "matrix.h"
#include <math.h>

#define max(a,b) ((a) > (b) ? (a) : (b))

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 2 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 2 inputs and 1 output.");
    }
        
    //gets size of variables
    const int length = mxGetDimensions(prhs[0])[0];
    
    //get value of input variables
    double * VIn = mxGetPr(prhs[0]);
    double * R = mxGetPr(prhs[1]);
    
    //setup outputs
    mwSize dims[] = {3,3,length};
    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *VOut = mxGetPr(plhs[0]);
       
    for(size_t i = 0; i < length; i++){
    
    	double* VI = &VIn[9*i];
        double* VO = &VOut[9*i];
    	
        //invert it
        VO[0] = (t[4]*t[8] - t[5]*t[7])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]);
        VO[1] = -(t[1]*t[8] - t[2]*t[7])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]);
        VO[2] = (t[1]*t[5] - t[2]*t[4])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]);
        VO[3] = -(t[3]*t[8] - t[5]*t[6])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]);
        VO[4] = (t[0]*t[8] - t[2]*t[6])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]); 
        VO[5] = -(t[0]*t[5] - t[2]*t[3])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]); 
        VO[6] = (t[3]*t[7] - t[4]*t[6])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]);
        VO[7] = -(t[0]*t[7] - t[1]*t[6])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]); 
        VO[8] = (t[0]*t[4] - t[1]*t[3])/(t[0]*t[4]*t[8] - t[0]*t[5]*t[7] - t[1]*t[3]*t[8] + t[1]*t[5]*t[6] + t[2]*t[3]*t[7] - t[2]*t[4]*t[6]);
    }
}
