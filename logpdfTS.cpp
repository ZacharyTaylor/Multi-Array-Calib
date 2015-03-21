/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using logpdfT(R,vR,tA,vtA,vB,vtB,RB,vRB,t)
 */ 

#include "mex.h"
#include "matrix.h"
#define _USE_MATH_DEFINES
#include <cmath>

#define max(a,b) ((a) > (b) ? (a) : (b))
#define OFFSET 0.001

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

void findScaleB(double* sB, double* tA, double* tB, double* RAM, double* RBM, double* RM, double* t){

    double x[3];
    x[0] = -RM[0]*t[0] - RM[1]*t[1] - RM[2]*t[2];
    x[1] = -RM[3]*t[0] - RM[4]*t[1] - RM[5]*t[2];
    x[2] = -RM[6]*t[0] - RM[7]*t[1] - RM[8]*t[2];
    
    sB[0] = (RAM[0]*t[0] + RAM[3]*t[1] + RAM[6]*t[2] - t[0] + tA[0]) / (RM[0]*tB[0] + RM[3]*tB[1] + RM[6]*tB[2]);
    sB[1] = (RAM[1]*t[0] + RAM[4]*t[1] + RAM[7]*t[2] - t[1] + tA[1]) / (RM[1]*tB[0] + RM[4]*tB[1] + RM[7]*tB[2]);
    sB[2] = (RAM[2]*t[0] + RAM[5]*t[1] + RAM[8]*t[2] - t[2] + tA[2]) / (RM[2]*tB[0] + RM[5]*tB[1] + RM[8]*tB[2]);
    
    sB[3] = -(RBM[0]*x[0] + RBM[3]*x[1] + RBM[6]*x[2] - x[0] - RM[0]*tA[0] - RM[1]*tA[1] - RM[2]*tA[2]) / tB[0];
    sB[4] = -(RBM[1]*x[0] + RBM[4]*x[1] + RBM[7]*x[2] - x[1] - RM[3]*tA[0] - RM[4]*tA[1] - RM[5]*tA[2]) / tB[1];
    sB[5] = -(RBM[2]*x[0] + RBM[5]*x[1] + RBM[8]*x[2] - x[2] - RM[6]*tA[0] - RM[7]*tA[1] - RM[8]*tA[2]) / tB[2];
}

void scaleElementVarB(double* err, double* base, double v, double* tA, double* tB, double* RA, double* RB, double* R, double* t){
    double temp[6];
    
    double RAM[9];
    double RBM[9];
    double RM[9];
    
	V2R(RA, RAM);
    V2R(RB,RBM);
    V2R(R,RM);
    
    findScaleB(temp,tA,tB,RAM,RBM,RM,t);
    for(size_t i = 0; i < 6; i++){
        err[i] += v*(temp[i] - base[i])*(temp[i] - base[i]);
    }
}

void combineScaleB(double* stB, double* svtB,
        double* tA, double* vtA,
        double* tB, double* vtB,
        double* RA, double* vRA,
        double* RB, double* vRB,
        double* R,  double* vR,
        double* t){
    
    double base[6];
    
    double RAM[9];
    double RBM[9];
    double RM[9];
    
    V2R(RA,RAM);
    V2R(RB,RBM);
    V2R(R,RM);
    
    findScaleB(base, tA, tB, RAM, RBM, RM, t);
    
    //add OFFSET to estimate variance
    double err[] = {0,0,0,0,0,0};

    for(size_t j = 0; j < 3; j++){
        tA[j] += OFFSET;
        scaleElementVarB(err,base, vtA[j], tA, tB, RA, RB, R, t);
        tA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        tB[j] += OFFSET;
        scaleElementVarB(err,base, vtB[j], tA, tB, RA, RB, R, t);
        tB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RA[j] += OFFSET;
        scaleElementVarB(err,base, vRA[j], tA, tB, RA, RB, R, t);
        RA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RB[j] += OFFSET;
        scaleElementVarB(err,base, vRB[j], tA, tB, RA, RB, R, t);
        RB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        R[j] += OFFSET;
        scaleElementVarB(err,base, vR[j], tA, tB, RA, RB, R, t);
        R[j] -= OFFSET;
    }
    
    //transform to variance
    for(size_t j = 0; j < 6; j++){
        err[j] = err[j]/(OFFSET*OFFSET);
        err[j] = 2/err[j];//2 as equations depend on each other alot
    }
    
    //combine estimates
    double s = 0;
    double sV = 0;
    for(size_t j = 0; j < 6; j++){
        s += base[j]*err[j];
        sV += err[j];
    }
    sV = (1/sV); 
    s = s*sV;
    
    //add to original
    for(size_t j = 0; j< 3; j++){
        stB[j] = s*tB[j];
        svtB[j] = s*vtB[j] + sV*tB[j];
    }
}

void findError(double* err, double* tA, double* tB, double* RAM, double* RBM, double* RM, double* t){
    
    double x[3];
    x[0] = -RM[0]*t[0] - RM[1]*t[1] - RM[2]*t[2];
    x[1] = -RM[3]*t[0] - RM[4]*t[1] - RM[5]*t[2];
    x[2] = -RM[6]*t[0] - RM[7]*t[1] - RM[8]*t[2];
    
    err[0] = RAM[0]*t[0] + RAM[3]*t[1] + RAM[6]*t[2] - t[0] - RM[0]*tB[0] - RM[3]*tB[1] - RM[6]*tB[2] + tA[0];
    err[1] = RAM[1]*t[0] + RAM[4]*t[1] + RAM[7]*t[2] - t[1] - RM[1]*tB[0] - RM[4]*tB[1] - RM[7]*tB[2] + tA[1];
    err[2] = RAM[2]*t[0] + RAM[5]*t[1] + RAM[8]*t[2] - t[2] - RM[2]*tB[0] - RM[5]*tB[1] - RM[8]*tB[2] + tA[2];
    
    err[3] = RBM[0]*x[0] + RBM[3]*x[1] + RBM[6]*x[2] - x[0] - RM[0]*tA[0] - RM[1]*tA[1] - RM[2]*tA[2] + tB[0];
    err[4] = RBM[1]*x[0] + RBM[4]*x[1] + RBM[7]*x[2] - x[1] - RM[3]*tA[0] - RM[4]*tA[1] - RM[5]*tA[2] + tB[1];
    err[5] = RBM[2]*x[0] + RBM[5]*x[1] + RBM[8]*x[2] - x[2] - RM[6]*tA[0] - RM[7]*tA[1] - RM[8]*tA[2] + tB[2];
} 

void errorElementVar(double* err, double* base, double v, double* tA, double* tB, double* RA, double* RB, double* R, double* t){
    double temp[6];
    
    double RAM[9];
    double RBM[9];
    double RM[9];
    
    V2R(RA,RAM);
    V2R(RB,RBM);
    V2R(R,RM);
    
    findError(temp,tA,tB,RAM,RBM,RM,t);
    for(size_t i = 0; i < 6; i++){
        err[i] += v*(temp[i] - base[i])*(temp[i] - base[i]);
    }
}

void findErrVar(double* err, double* verr,
        double* tA, double* vtA,
        double* tB, double* vtB,
        double* RA, double* vRA,
        double* RB, double* vRB,
        double* R,  double* vR,
        double* t){
        
    double RAM[9];
    double RBM[9];
    double RM[9];
    
    V2R(RA,RAM);
    V2R(RB,RBM);
    V2R(R,RM);
    
    findError(err, tA, tB, RAM, RBM, RM, t);
    
    //add OFFSET to estimate variance
    for(size_t j = 0; j < 6; j++){
        verr[j] = 0;
    }
    
    for(size_t j = 0; j < 3; j++){
        tA[j] += OFFSET;
        errorElementVar(verr,err, vtA[j], tA, tB, RA, RB, R, t);
        tA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        tB[j] += OFFSET;
        errorElementVar(verr,err, vtB[j], tA, tB, RA, RB, R, t);
        tB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RA[j] += OFFSET;
        errorElementVar(verr,err, vRA[j], tA, tB, RA, RB, R, t);
        RA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RB[j] += OFFSET;
        errorElementVar(verr,err, vRB[j], tA, tB, RA, RB, R, t);
        RB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        R[j] += OFFSET;
        errorElementVar(verr,err, vR[j], tA, tB, RA, RB, R, t);
        R[j] -= OFFSET;
    }
    
    //transform to variance
    for(size_t j = 0; j < 6; j++){
        verr[j] = verr[j]/(OFFSET*OFFSET);
    }
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
    	
        double stB[3];
        double svtB[3];
        double err[6];
        double verr[6];
        
        combineScaleB(stB,svtB,tA,vtA,tB,vtB,RA,vRA,RB,vRB,R,vR,t);
        findErrVar(err,verr,tA,vtA,stB,svtB,RA,vRA,RB,vRB,R,vR,t);
		            
        //find exponential exponent
        double eExp1 = -0.5*(err[0]*err[0]/verr[0] + err[1]*err[1]/verr[1] + err[2]*err[2]/verr[2]);
        //find part before exponential
        double bExp1 = -log(sqrt(8*M_PI*M_PI*M_PI*verr[0]*verr[1]*verr[2]));
        
        //find exponential exponent
        double eExp2 = -0.5*(err[3]*err[3]/verr[3] + err[4]*err[4]/verr[4] + err[5]*err[5]/verr[5]);
        //find part before exponential
        double bExp2 = -log(sqrt(8*M_PI*M_PI*M_PI*verr[3]*verr[4]*verr[5]));
        
        //finding log likelihood
        logl[0] += (bExp1 + eExp1 + bExp2 + eExp2)/2;
    }
}
