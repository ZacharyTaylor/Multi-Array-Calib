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
    
void findError(double* err, double* tA, double* tB, double* RA, double* RB, double* R, double* t){
    
    double x[3];
    x[0] = -R[0]*t[0] - R[1]*t[1] - R[2]*t[2];
    x[1] = -R[3]*t[0] - R[4]*t[1] - R[5]*t[2];
    x[2] = -R[6]*t[0] - R[7]*t[1] - R[8]*t[2];
    
    err[0] = RA[0]*t[0] + RA[3]*t[1] + RA[6]*t[2] - t[0] - R[0]*tB[0] - R[3]*tB[1] - R[6]*tB[2] + tA[0];
    err[1] = RA[1]*t[0] + RA[4]*t[1] + RA[7]*t[2] - t[1] - R[1]*tB[0] - R[4]*tB[1] - R[7]*tB[2] + tA[1];
    err[2] = RA[2]*t[0] + RA[5]*t[1] + RA[8]*t[2] - t[2] - R[2]*tB[0] - R[5]*tB[1] - R[8]*tB[2] + tA[2];
    
    err[3] = RB[0]*x[0] + RB[3]*x[1] + RB[6]*x[2] - x[0] - R[0]*tA[0] - R[1]*tA[1] - R[2]*tA[2] + tB[0];
    err[4] = RB[1]*x[0] + RB[4]*x[1] + RB[7]*x[2] - x[1] - R[3]*tA[0] - R[4]*tA[1] - R[5]*tA[2] + tB[1];
    err[5] = RB[2]*x[0] + RB[5]*x[1] + RB[8]*x[2] - x[2] - R[6]*tA[0] - R[7]*tA[1] - R[8]*tA[2] + tB[2];
}   

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 11 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 11 inputs and 1 output.");
    }
        
    //gets size of variables
    const int* length = mxGetDimensions(prhs[2]);
    const size_t n = length[1];
    
    //get value of input variables
    double * R = mxGetPr(prhs[0]);
    double * vR = mxGetPr(prhs[1]);
    double * tAIn = mxGetPr(prhs[2]);
    double * vtAIn = mxGetPr(prhs[3]);
    double * RAIn = mxGetPr(prhs[4]);
    double * vRAIn = mxGetPr(prhs[5]);
    double * tBIn = mxGetPr(prhs[6]);
    double * vtBIn = mxGetPr(prhs[7]);
    double * RBIn = mxGetPr(prhs[8]);
    double * vRBIn = mxGetPr(prhs[9]);
    double * t = mxGetPr(prhs[10]);
    
    //setup outputs
    plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double *logl = mxGetPr(plhs[0]);
    logl[0] = 0;
       
    for(size_t i = 0; i < n; i++){
		
    	double* tA = &tAIn[3*i];
    	double* tB = &tBIn[3*i];
        double* RA = &RAIn[3*i];
        double* RB = &RBIn[3*i];
        double* vtA = &vtAIn[3*i];
    	double* vtB = &vtBIn[3*i];
        double* vRA = &vRAIn[3*i];
        double* vRB = &vRBIn[3*i];
    	       
        //find error
        double err[6];
        double RAM[9];
        double RBM[9];
        double RM[9];
		
        V2R(RA,RAM);
        V2R(RB,RBM);
        V2R(R,RM);
		
        findError(err,tA,tB,RAM,RBM,RM,t);

        //add offset to estimate variance
        double offset = 0.0001;
        double errOff[] = {0,0,0,0,0,0};

        for(size_t j = 0; j < 3; j++){
            tA[j] += offset;
            double temp[6];
            findError(temp,tA,tB,RAM,RBM,RM,t);
            tA[j] -= offset;
            
            errOff[0] += vtA[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vtA[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vtA[j]*(temp[2] - err[2])*(temp[2] - err[2]);
            errOff[3] += vtA[j]*(temp[3] - err[3])*(temp[3] - err[3]);
            errOff[4] += vtA[j]*(temp[4] - err[4])*(temp[4] - err[4]);
            errOff[5] += vtA[j]*(temp[5] - err[5])*(temp[5] - err[5]);
        }
        
        for(size_t j = 0; j < 3; j++){
            tB[j] += offset;
            double temp[6];
            findError(temp,tA,tB,RAM,RBM,RM,t);
            tB[j] -= offset;
            
            errOff[0] += vtB[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vtB[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vtB[j]*(temp[2] - err[2])*(temp[2] - err[2]);
            errOff[3] += vtB[j]*(temp[3] - err[3])*(temp[3] - err[3]);
            errOff[4] += vtB[j]*(temp[4] - err[4])*(temp[4] - err[4]);
            errOff[5] += vtB[j]*(temp[5] - err[5])*(temp[5] - err[5]);
        }
        
        for(size_t j = 0; j < 3; j++){
            RA[j] += offset;
            V2R(RA,RAM);
            double temp[6];
            findError(temp,tA,tB,RAM,RBM,RM,t);
            RA[j] -= offset;
            
            errOff[0] += vRA[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vRA[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vRA[j]*(temp[2] - err[2])*(temp[2] - err[2]);
            errOff[3] += vRA[j]*(temp[3] - err[3])*(temp[3] - err[3]);
            errOff[4] += vRA[j]*(temp[4] - err[4])*(temp[4] - err[4]);
            errOff[5] += vRA[j]*(temp[5] - err[5])*(temp[5] - err[5]);
        }
        V2R(RA,RAM);
        for(size_t j = 0; j < 3; j++){
            RB[j] += offset;
            V2R(RB,RBM);
            double temp[6];
            findError(temp,tA,tB,RAM,RBM,RM,t);
            RB[j] -= offset;
            
            errOff[0] += vRB[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vRB[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vRB[j]*(temp[2] - err[2])*(temp[2] - err[2]);
            errOff[3] += vRB[j]*(temp[3] - err[3])*(temp[3] - err[3]);
            errOff[4] += vRB[j]*(temp[4] - err[4])*(temp[4] - err[4]);
            errOff[5] += vRB[j]*(temp[5] - err[5])*(temp[5] - err[5]);
        }
        V2R(RB,RBM);
        for(size_t j = 0; j < 3; j++){
            R[j] += offset;
            V2R(R,RM);
            double temp[6];
            findError(temp,tA,tB,RAM,RBM,RM,t);
            R[j] -= offset;
            
            errOff[0] += vR[j]*(temp[0] - err[0])*(temp[0] - err[0]);
            errOff[1] += vR[j]*(temp[1] - err[1])*(temp[1] - err[1]);
            errOff[2] += vR[j]*(temp[2] - err[2])*(temp[2] - err[2]);
            errOff[3] += vR[j]*(temp[3] - err[3])*(temp[3] - err[3]);
            errOff[4] += vR[j]*(temp[4] - err[4])*(temp[4] - err[4]);
            errOff[5] += vR[j]*(temp[5] - err[5])*(temp[5] - err[5]);
        }
        
        //transform to variance
        errOff[0] = errOff[0]/(offset*offset);
		errOff[1] = errOff[1]/(offset*offset);
		errOff[2] = errOff[2]/(offset*offset);
        errOff[3] = errOff[3]/(offset*offset);
		errOff[4] = errOff[4]/(offset*offset);
		errOff[5] = errOff[5]/(offset*offset);
            
        //find exponential exponent
        double eExp1 = -0.5*(err[0]*err[0]/errOff[0] + err[1]*err[1]/errOff[1] + err[2]*err[2]/errOff[2]);
        //find part before exponential
        double bExp1 = -log(sqrt(8*M_PI*M_PI*M_PI*errOff[0]*errOff[1]*errOff[2]));
        
        //find exponential exponent
        double eExp2 = -0.5*(err[3]*err[3]/errOff[3] + err[4]*err[4]/errOff[4] + err[5]*err[5]/errOff[5]);
        //find part before exponential
        double bExp2 = -log(sqrt(8*M_PI*M_PI*M_PI*errOff[3]*errOff[4]*errOff[5]));
        
        //finding log likelihood
        logl[0] += bExp1 + eExp1 + bExp2 + eExp2;
    }
}
