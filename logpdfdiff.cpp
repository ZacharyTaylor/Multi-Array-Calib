/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using cProb(err, VA, VB, rot, diff)
 */ 

#include "mex.h"
#include "matrix.h"
#define _USE_MATH_DEFINES
#include <cmath>

#define max(a,b) ((a) > (b) ? (a) : (b))

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 4 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 4 inputs and 1 output.");
    }
        
    //gets size of variables
    const int* length = mxGetDimensions(prhs[0]);
    const size_t loopFor = length[1];
    
    //get value of input variables
    double * err = mxGetPr(prhs[0]);
    double * errO = mxGetPr(prhs[1]);
    double * VAIn = mxGetPr(prhs[2]);
    double * VBIn = mxGetPr(prhs[3]);
    double * R = mxGetPr(prhs[4]);
    double * diff = mxGetPr(prhs[5]);
    
    //setup outputs
    plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double *logl = mxGetPr(plhs[0]);
    logl[0] = 0;
    
    double V[9];
    double invV[9];
    
    for(size_t i = 0; i < loopFor; i++){
    
    	double* VA = &VAIn[3*i];
    	double* VB = &VBIn[3*i];
    	double* e = &err[3*i];
    	
        //calc variance
    	V[0] = R[0]*R[0]*VA[0] + R[3]*R[3]*VA[1] + R[6]*R[6]*VA[2] + VB[0];
        V[1] = R[0]*R[1]*VA[0] + R[3]*R[4]*VA[1] + R[6]*R[7]*VA[2];
        V[2] = R[0]*R[2]*VA[0] + R[3]*R[5]*VA[1] + R[6]*R[8]*VA[2];
        V[3] = R[0]*R[1]*VA[0] + R[3]*R[4]*VA[1] + R[6]*R[7]*VA[2];
        V[4] = R[1]*R[1]*VA[0] + R[4]*R[4]*VA[1] + R[7]*R[7]*VA[2] + VB[1];
        V[5] = R[1]*R[2]*VA[0] + R[4]*R[5]*VA[1] + R[7]*R[8]*VA[2];
        V[6] = R[0]*R[2]*VA[0] + R[3]*R[5]*VA[1] + R[6]*R[8]*VA[2];
        V[7] = R[1]*R[2]*VA[0] + R[4]*R[5]*VA[1] + R[7]*R[8]*VA[2];
        V[8] = R[2]*R[2]*VA[0] + R[5]*R[5]*VA[1] + R[8]*R[8]*VA[2] + VB[2];
        
        //find the determinate of the variance
        double det = V[0]*V[4]*V[8] - V[0]*V[5]*V[7] - V[1]*V[3]*V[8] + V[1]*V[5]*V[6] + V[2]*V[3]*V[7] - V[2]*V[4]*V[6];
        
        //invert variance
        invV[0] = (V[4]*V[8] - V[5]*V[7])/det;
        invV[1] = (V[2]*V[7] - V[1]*V[8])/det;
        invV[2] = (V[1]*V[5] - V[2]*V[4])/det;
        invV[3] = (V[5]*V[6] - V[3]*V[8])/det;
        invV[4] = (V[0]*V[8] - V[2]*V[6])/det;
        invV[5] = (V[2]*V[3] - V[0]*V[5])/det;
        invV[6] = (V[3]*V[7] - V[4]*V[6])/det;
        invV[7] = (V[1]*V[6] - V[0]*V[7])/det;
        invV[8] = (V[0]*V[4] - V[1]*V[3])/det;
        
        //find exponential exponent
        double eExp = -0.5*(e[0]*(invV[0]*e[0] + invV[1]*e[1] + invV[2]*e[2]) + e[1]*(invV[3]*e[0] + invV[4]*e[1] + invV[5]*e[2]) + e[2]*(invV[6]*e[0] + invV[7]*e[1] + invV[8]*e[2]));
        //find part before exponential
        double bExp = -log(sqrt(8*M_PI*M_PI*M_PI*det));
        
        //finding log likelihood
        logl[0] += bExp + eExp;
    }
}
