/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using cProb(V, err)
 */ 

#include "mex.h"
#include "matrix.h"
#include <math.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 2 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 2 inputs and 1 output.");
    }
        
    //gets size of variables
    const int length = mxGetDimensions(prhs[1])[0];
    int depth = 1;
    if(mxGetNumberOfDimensions(prhs[1]) > 2){
        depth = mxGetDimensions(prhs[1])[2];
    }
    
    //get value of input variables
    double * VIn = mxGetPr(prhs[0]);
    double * eIn = mxGetPr(prhs[1]);
    
    //setup outputs
    mwSize dims[] = {length,3,depth};
    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *eOut = mxGetPr(plhs[0]);
       
    for(size_t i = 0; i < length; i++){
        for(size_t j = 0; j < depth; j++){
    
            double* VI = &VIn[9*i + 9*length*j];
            double* eI = &eIn[3*i + 3*length*j];
            double* eO = &eOut[3*i + 3*length*j];

            eO[0] = VI[0]*eI[0] + VI[3]*eI[1] + VI[6]*eI[2];
            eO[1] = VI[1]*eI[0] + VI[4]*eI[1] + VI[7]*eI[2];
            eO[2] = VI[2]*eI[0] + VI[5]*eI[1] + VI[8]*eI[2];
        }
    }
}
