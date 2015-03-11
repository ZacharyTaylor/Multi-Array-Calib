/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using logpdfT(R,vR,tA,vtA,vB,vtB,RB,vRB,t)
 */ 

#include "mex.h"
#include "matrix.h"
#define _USE_MATH_DEFINES
#include <cmath>

#define max(a,b) ((a) > (b) ? (a) : (b))

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
    
void findError(double* err, double* tA, double* tB, double* RB, double* R, double* t){
    err[0] = t[0] - tB[0] + R[0]*tA[0] + R[3]*tA[1] + R[6]*tA[2] - RB[0]*t[0] - RB[3]*t[1] - RB[6]*t[2];
    err[1] = t[1] - tB[1] + R[1]*tA[0] + R[4]*tA[1] + R[7]*tA[2] - RB[1]*t[0] - RB[4]*t[1] - RB[7]*t[2];
    err[2] = t[2] - tB[2] + R[2]*tA[0] + R[5]*tA[1] + R[8]*tA[2] - RB[2]*t[0] - RB[5]*t[1] - RB[8]*t[2];
}   

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 9 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 9 inputs and 1 output.");
    }
        
    //gets size of variables
    const int* length = mxGetDimensions(prhs[2]);
    const size_t n = length[1];
    
    //get value of input variables
    double * R = mxGetPr(prhs[0]);
    double * vR = mxGetPr(prhs[1]);
    double * tAIn = mxGetPr(prhs[2]);
    double * vtAIn = mxGetPr(prhs[3]);
    double * tBIn = mxGetPr(prhs[4]);
    double * vtBIn = mxGetPr(prhs[5]);
    double * RBIn = mxGetPr(prhs[6]);
    double * vRBIn = mxGetPr(prhs[7]);
    double * t = mxGetPr(prhs[8]);
    
    //setup outputs
    plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double *logl = mxGetPr(plhs[0]);
    logl[0] = 0;
       
    for(size_t i = 0; i < n; i++){
		
    	double* tA = &tAIn[3*i];
    	double* tB = &tBIn[3*i];
        double* RB = &RBIn[3*i];
        double* vtA = &vtAIn[3*i];
    	double* vtB = &vtBIn[3*i];
        double* vRB = &vRBIn[3*i];
    	       
        //find error
        double err[3];
        double RBM[9];
        double RM[9];
		
        V2R(RB,RBM);
        V2R(R,RM);
		
        findError(err,tA,tB,RBM,RM,t);

        //add offset to estimate variance
        double offset = 0.0001;
        double errOff[] = {0,0,0};

        for(size_t j = 0; j < 3; j++){
            tA[j] += offset;
            double temp[3];
            findError(temp,tA,tB,RBM,RM,t);
            tA[j] -= offset;
            
            errOff[0] += vtA[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vtA[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vtA[j]*(temp[2] - err[2])*(temp[2] - err[2]);
        }
        
        for(size_t j = 0; j < 3; j++){
            tB[j] += offset;
            double temp[3];
            findError(temp,tA,tB,RBM,RM,t);
            tB[j] -= offset;
            
            errOff[0] += vtB[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vtB[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vtB[j]*(temp[2] - err[2])*(temp[2] - err[2]);
        }
        
        for(size_t j = 0; j < 3; j++){
            RB[j] += offset;
            V2R(RB,RBM);
            double temp[3];
            findError(temp,tA,tB,RBM,RM,t);
            RB[j] -= offset;
            
            errOff[0] += vRB[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vRB[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vRB[j]*(temp[2] - err[2])*(temp[2] - err[2]);
        }
        V2R(RB,RBM);
        for(size_t j = 0; j < 3; j++){
            R[j] += offset;
            V2R(R,RM);
            double temp[3];
            findError(temp,tA,tB,RBM,RM,t);
            R[j] -= offset;
            
            errOff[0] += vR[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vR[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vR[j]*(temp[2] - err[2])*(temp[2] - err[2]);
        }
        
        //transform to variance
        errOff[0] = errOff[0]/(offset*offset);
		errOff[1] = errOff[1] / (offset*offset);
		errOff[2] = errOff[2] / (offset*offset);
            
        //find exponential exponent
        double eExp = -0.5*(err[0]*err[0]/errOff[0] + err[1]*err[1]/errOff[1] + err[2]*err[2]/errOff[2]);
        //find part before exponential
        double bExp = -log(sqrt(8*M_PI*M_PI*M_PI*errOff[0]*errOff[1]*errOff[2]));
        
        //finding log likelihood
        logl[0] += bExp + eExp;
    }
}
