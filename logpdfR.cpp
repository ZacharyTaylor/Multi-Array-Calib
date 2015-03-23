/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using cProb(err, VA, VB, rot)
 */ 

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
    if (nrhs != 6 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 6 inputs and 1 output.");
    }
        
    //gets size of variables
    const int* length = mxGetDimensions(prhs[0]);
    const size_t loopFor = length[1];
    
    //get value of input variables
    double * RAIn = mxGetPr(prhs[0]);
    double * RBIn = mxGetPr(prhs[1]);
    double * VAIn = mxGetPr(prhs[2]);
    double * VBIn = mxGetPr(prhs[3]);
    double * Rs = mxGetPr(prhs[4]);
    double * Re = mxGetPr(prhs[5]);
    
    //setup outputs
    plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double *logl = mxGetPr(plhs[0]);
    logl[0] = 0;
    
    double V[9];
    double invV[9];
    
    for(size_t i = 0; i < loopFor; i++){
    
        double* RA = &RAIn[3*i];
    	double* RB = &RBIn[3*i];
    	double* VA = &VAIn[3*i];
    	double* VB = &VBIn[3*i];
    	
        //find rotation matrix
        double RSM[9];
        double REM[9];
        double R[9];

        V2R(Rs,RSM);
        V2R(Re,REM);

        R[0] = RSM[0]*REM[0] + RSM[1]*REM[1] + RSM[2]*REM[2];
        R[1] = RSM[3]*REM[0] + RSM[4]*REM[1] + RSM[5]*REM[2];
        R[2] = RSM[6]*REM[0] + RSM[7]*REM[1] + RSM[8]*REM[2];
        R[3] = RSM[0]*REM[3] + RSM[1]*REM[4] + RSM[2]*REM[5];
        R[4] = RSM[3]*REM[3] + RSM[4]*REM[4] + RSM[5]*REM[5];
        R[5] = RSM[6]*REM[3] + RSM[7]*REM[4] + RSM[8]*REM[5];
        R[6] = RSM[0]*REM[6] + RSM[1]*REM[7] + RSM[2]*REM[8];
        R[7] = RSM[3]*REM[6] + RSM[4]*REM[7] + RSM[5]*REM[8];
        R[8] = RSM[6]*REM[6] + RSM[7]*REM[7] + RSM[8]*REM[8];
        
        //calc error
        double e[3];
        e[0] = RA[0] - RB[0]*R[0] - RB[1]*R[3] - RB[2]*R[6];
        e[1] = RA[1] - RB[0]*R[1] - RB[1]*R[4] - RB[2]*R[7];
        e[2] = RA[2] - RB[0]*R[2] - RB[1]*R[5] - RB[2]*R[8];
        
        //calc variance
    	V[0] = R[0]*R[0]*VB[0] + R[3]*R[3]*VB[1] + R[6]*R[6]*VB[2] + VA[0];
        V[1] = R[0]*R[1]*VB[0] + R[3]*R[4]*VB[1] + R[6]*R[7]*VB[2];
        V[2] = R[0]*R[2]*VB[0] + R[3]*R[5]*VB[1] + R[6]*R[8]*VB[2];
        V[3] = R[0]*R[1]*VB[0] + R[3]*R[4]*VB[1] + R[6]*R[7]*VB[2];
        V[4] = R[1]*R[1]*VB[0] + R[4]*R[4]*VB[1] + R[7]*R[7]*VB[2] + VA[1];
        V[5] = R[1]*R[2]*VB[0] + R[4]*R[5]*VB[1] + R[7]*R[8]*VB[2];
        V[6] = R[0]*R[2]*VB[0] + R[3]*R[5]*VB[1] + R[6]*R[8]*VB[2];
        V[7] = R[1]*R[2]*VB[0] + R[4]*R[5]*VB[1] + R[7]*R[8]*VB[2];
        V[8] = R[2]*R[2]*VB[0] + R[5]*R[5]*VB[1] + R[8]*R[8]*VB[2] + VA[2];
        
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
