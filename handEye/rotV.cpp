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
        for(size_t j = 0; j < depth; j++){
    
            double* VI = &VIn[3*i + ];
            double* VO = &VOut[9*i];

            //calc variance
            VO[0] = R[0]*R[0]*VI[0] + R[3]*R[3]*VI[1] + R[6]*R[6]*VI[2];
            VO[1] = R[1]*R[0]*VI[0] + R[4]*R[3]*VI[1] + R[7]*R[6]*VI[2];
            VO[2] = R[2]*R[0]*VI[0] + R[5]*R[3]*VI[1] + R[8]*R[6]*VI[2];
            VO[3] = R[0]*R[1]*VI[0] + R[3]*R[4]*VI[1] + R[6]*R[7]*VI[2];
            VO[4] = R[1]*R[1]*VI[0] + R[4]*R[4]*VI[1] + R[7]*R[7]*VI[2];
            VO[5] = R[2]*R[1]*VI[0] + R[5]*R[4]*VI[1] + R[8]*R[7]*VI[2];
            VO[6] = R[0]*R[2]*VI[0] + R[3]*R[5]*VI[1] + R[6]*R[8]*VI[2];
            VO[7] = R[1]*R[2]*VI[0] + R[4]*R[5]*VI[1] + R[7]*R[8]*VI[2];
            VO[8] = R[2]*R[2]*VI[0] + R[5]*R[5]*VI[1] + R[8]*R[8]*VI[2];
        }
    }
}
