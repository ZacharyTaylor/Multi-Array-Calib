/*Small program to quickly get probablity as matlab cant loop to save itself
 * call using logpdfT(R,vR,tA,vtA,vB,vtB,RB,vRB,t)
 */ 

#include "logpdfTG.h"

__device__ void V2R(const double* const vect, double* const R){
    
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

__device__ void findScaleB(double* sB, double* tA, double* tB, double* RA, double* RB, double* Rs, double* Re, double* t){

    double RAM[9];
    double RBM[9];
    double RSM[9];
    double REM[9];
    double RM[9];
    
    V2R(RA,RAM);
    V2R(RB,RBM);
    V2R(Rs,RSM);
    V2R(Re,REM);
    
    RM[0] = RSM[0]*REM[0] + RSM[1]*REM[1] + RSM[2]*REM[2];
    RM[1] = RSM[3]*REM[0] + RSM[4]*REM[1] + RSM[5]*REM[2];
    RM[2] = RSM[6]*REM[0] + RSM[7]*REM[1] + RSM[8]*REM[2];
    RM[3] = RSM[0]*REM[3] + RSM[1]*REM[4] + RSM[2]*REM[5];
    RM[4] = RSM[3]*REM[3] + RSM[4]*REM[4] + RSM[5]*REM[5];
    RM[5] = RSM[6]*REM[3] + RSM[7]*REM[4] + RSM[8]*REM[5];
    RM[6] = RSM[0]*REM[6] + RSM[1]*REM[7] + RSM[2]*REM[8];
    RM[7] = RSM[3]*REM[6] + RSM[4]*REM[7] + RSM[5]*REM[8];
    RM[8] = RSM[6]*REM[6] + RSM[7]*REM[7] + RSM[8]*REM[8];
    
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

__device__ void scaleElementVarB(double* err, double* base, double v, double* tA, double* tB, double* RA, double* RB, double* Rs, double* Re, double* t){
    double temp[6];
    
    findScaleB(temp,tA,tB,RA,RB,Rs,Re,t);
    for(size_t i = 0; i < 6; i++){
        err[i] += v*(temp[i] - base[i])*(temp[i] - base[i]);
    }
}

__global__ void  combineScaleB(double* stBPtr, double* svtBPtr,
        double* tAPtr, double* vtAPtr,
        double* tBPtr, double* vtBPtr,
        double* RAPtr, double* vRAPtr,
        double* RBPtr, double* vRBPtr,
        double* RPtr, double* vRPtr,
        double* t, size_t n){
    
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;

	if(i >= n){
		return;
	}
    
    double* tA = &tAPtr[3*i];
    double* tB = &tBPtr[3*i];
    double* RA = &RAPtr[3*i];
    double* RB = &RBPtr[3*i];
    double* vtA = &vtAPtr[3*i];
    double* vtB = &vtBPtr[3*i];
    double* vRA = &vRAPtr[3*i];
    double* vRB = &vRBPtr[3*i];

	double* stB = &stBPtr[3 * i];
	double* svtB = &svtBPtr[3 * i];

    double* Rs = &RPtr[0];
    double* Re = &RPtr[3];
    double* vRs = &vRPtr[0];
    double* vRe = &vRPtr[3];
    
    double base[6];
    
    findScaleB(base, tA, tB, RA, RB, Rs, Re, t);
    
    //add OFFSET to estimate variance
    double err[] = {0,0,0,0,0,0};

    for(size_t j = 0; j < 3; j++){
        tA[j] += OFFSET;
        scaleElementVarB(err,base, vtA[j], tA, tB, RA, RB, Rs, Re, t);
        tA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        tB[j] += OFFSET;
        scaleElementVarB(err,base, vtB[j], tA, tB, RA, RB, Rs, Re, t);
        tB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RA[j] += OFFSET;
        scaleElementVarB(err,base, vRA[j], tA, tB, RA, RB, Rs, Re, t);
        RA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RB[j] += OFFSET;
        scaleElementVarB(err,base, vRB[j], tA, tB, RA, RB, Rs, Re, t);
        RB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        Rs[j] += OFFSET;
        scaleElementVarB(err,base, vRs[j], tA, tB, RA, RB, Rs, Re, t);
        Rs[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        Re[j] += OFFSET;
        scaleElementVarB(err,base, vRe[j], tA, tB, RA, RB, Rs, Re, t);
        Re[j] -= OFFSET;
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
        svtB[j] = s*s*vtB[j] + sV*tB[j]*tB[j];
    }
}

__device__ void findError(double* err, double* tA, double* tB, double* RA, double* RB, double* Rs, double* Re, double* t){
    
    double RAM[9];
    double RBM[9];
    double RSM[9];
    double REM[9];
    double RM[9];
    
    V2R(RA,RAM);
    V2R(RB,RBM);
    V2R(Rs,RSM);
    V2R(Re,REM);
    
    RM[0] = RSM[0]*REM[0] + RSM[1]*REM[1] + RSM[2]*REM[2];
    RM[1] = RSM[3]*REM[0] + RSM[4]*REM[1] + RSM[5]*REM[2];
    RM[2] = RSM[6]*REM[0] + RSM[7]*REM[1] + RSM[8]*REM[2];
    RM[3] = RSM[0]*REM[3] + RSM[1]*REM[4] + RSM[2]*REM[5];
    RM[4] = RSM[3]*REM[3] + RSM[4]*REM[4] + RSM[5]*REM[5];
    RM[5] = RSM[6]*REM[3] + RSM[7]*REM[4] + RSM[8]*REM[5];
    RM[6] = RSM[0]*REM[6] + RSM[1]*REM[7] + RSM[2]*REM[8];
    RM[7] = RSM[3]*REM[6] + RSM[4]*REM[7] + RSM[5]*REM[8];
    RM[8] = RSM[6]*REM[6] + RSM[7]*REM[7] + RSM[8]*REM[8];
        
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

__device__ void errorElementVar(double* err, double* base, double v, double* tA, double* tB, double* RA, double* RB, double* Rs, double* Re, double* t){
    double temp[6];
        
    findError(temp,tA,tB,RA,RB,Rs,Re,t);
    for(size_t i = 0; i < 6; i++){
        err[i] += v*(temp[i] - base[i])*(temp[i] - base[i]);
    }
}

__global__ void findErrVar(double* loglPtr,
        double* tAPtr, double* vtAPtr,
        double* tBPtr, double* vtBPtr,
        double* RAPtr, double* vRAPtr,
        double* RBPtr, double* vRBPtr,
        double* RPtr, double* vRPtr,
        double* t, size_t n){
    
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;

	if(i >= n){
		return;
	}
    
    double* tA = &tAPtr[3*i];
    double* tB = &tBPtr[3*i];
    double* RA = &RAPtr[3*i];
    double* RB = &RBPtr[3*i];
    double* vtA = &vtAPtr[3*i];
    double* vtB = &vtBPtr[3*i];
    double* vRA = &vRAPtr[3*i];
    double* vRB = &vRBPtr[3*i];

    double* Rs = &RPtr[0];
    double* Re = &RPtr[3];
    double* vRs = &vRPtr[0];
    double* vRe = &vRPtr[3];
    
    double* logl = &loglPtr[i];
    
    double err[6];
    double verr[6];
    
    findError(err, tA, tB, RA, RB, Rs, Re, t);
    
    //add OFFSET to estimate variance
    for(size_t j = 0; j < 6; j++){
        verr[j] = 0;
    }
    
    for(size_t j = 0; j < 3; j++){
        tA[j] += OFFSET;
        errorElementVar(verr,err, vtA[j], tA, tB, RA, RB, Rs, Re, t);
        tA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        tB[j] += OFFSET;
        errorElementVar(verr,err, vtB[j], tA, tB, RA, RB, Rs, Re, t);
        tB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RA[j] += OFFSET;
        errorElementVar(verr,err, vRA[j], tA, tB, RA, RB, Rs, Re, t);
        RA[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        RB[j] += OFFSET;
        errorElementVar(verr,err, vRB[j], tA, tB, RA, RB, Rs, Re, t);
        RB[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        Rs[j] += OFFSET;
        errorElementVar(verr,err, vRs[j], tA, tB, RA, RB, Rs, Re, t);
        Rs[j] -= OFFSET;
    }
    for(size_t j = 0; j < 3; j++){
        Re[j] += OFFSET;
        errorElementVar(verr,err, vRe[j], tA, tB, RA, RB, Rs, Re, t);
        Re[j] -= OFFSET;
    }
    
    //transform to variance
    for(size_t j = 0; j < 6; j++){
        verr[j] = verr[j]/(OFFSET*OFFSET);
    }
    
    //find exponential exponent
    double eExp1 = -0.5*(err[0]*err[0]/verr[0] + err[1]*err[1]/verr[1] + err[2]*err[2]/verr[2]);
    //find part before exponential
    double bExp1 = -log(sqrt(8*PI*PI*PI*verr[0]*verr[1]*verr[2]));

    //find exponential exponent
    double eExp2 = -0.5*(err[3]*err[3]/verr[3] + err[4]*err[4]/verr[4] + err[5]*err[5]/verr[5]);
    //find part before exponential
    double bExp2 = -log(sqrt(8*PI*PI*PI*verr[3]*verr[4]*verr[5]));

    //finding log likelihood
    logl[0] = (bExp1 + eExp1 + bExp2 + eExp2)/2;
}  

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 12 || (nlhs != 1 && nlhs != 0)) {
      mexErrMsgIdAndTxt("CPROB:BadNArgs", 
                        "Need 12 inputs and 1 output.");
    }
    
    //get value of input variables
    mxGPUArray const * RIn = mxGPUCreateFromMxArray(prhs[0]);
    double* RPtr = (double*)(mxGPUGetDataReadOnly(RIn));
    mxGPUArray const * vRIn = mxGPUCreateFromMxArray(prhs[1]);
    double* vRPtr = (double*)(mxGPUGetDataReadOnly(vRIn));
    mxGPUArray const * tAIn = mxGPUCreateFromMxArray(prhs[2]);
    double* tAPtr = (double*)(mxGPUGetDataReadOnly(tAIn));
    mxGPUArray const * vtAIn = mxGPUCreateFromMxArray(prhs[3]);
    double* vtAPtr = (double*)(mxGPUGetDataReadOnly(vtAIn));
    mxGPUArray const * RAIn = mxGPUCreateFromMxArray(prhs[4]);
    double* RAPtr = (double*)(mxGPUGetDataReadOnly(RAIn));
    mxGPUArray const * vRAIn = mxGPUCreateFromMxArray(prhs[5]);
    double* vRAPtr = (double*)(mxGPUGetDataReadOnly(vRAIn));
    mxGPUArray const * tBIn = mxGPUCreateFromMxArray(prhs[6]);
    double* tBPtr = (double*)(mxGPUGetDataReadOnly(tBIn));
    mxGPUArray const * vtBIn = mxGPUCreateFromMxArray(prhs[7]);
    double* vtBPtr = (double*)(mxGPUGetDataReadOnly(vtBIn));
    mxGPUArray const * RBIn = mxGPUCreateFromMxArray(prhs[8]);
    double* RBPtr = (double*)(mxGPUGetDataReadOnly(RBIn));
    mxGPUArray const * vRBIn = mxGPUCreateFromMxArray(prhs[9]);
    double* vRBPtr = (double*)(mxGPUGetDataReadOnly(vRBIn));
    mxGPUArray const * tIn = mxGPUCreateFromMxArray(prhs[10]);
    double* tPtr = (double*)(mxGPUGetDataReadOnly(tIn));
    double * s = mxGetPr(prhs[11]);
    
    //gets size of variables
    const size_t n = mxGPUGetDimensions(tAIn)[1];
    
    //setup outputs
	mwSize outSize[] = {n,1};
    mxGPUArray* outMat = mxGPUCreateGPUArray(2, outSize, mxDOUBLE_CLASS, mxREAL, MX_GPU_DO_NOT_INITIALIZE);
	plhs[0] = mxGPUCreateMxArrayOnGPU(outMat);
    double* outPtr = (double*)(mxGPUGetData(outMat));
    
    //setup storage
    outSize[0] = 3;
    outSize[1] = n;
    mxGPUArray* stBIn = mxGPUCreateGPUArray(2, outSize, mxDOUBLE_CLASS, mxREAL, MX_GPU_DO_NOT_INITIALIZE);
    double* stBPtr = (double*)(mxGPUGetData(stBIn));
    mxGPUArray* svtBIn = mxGPUCreateGPUArray(2, outSize, mxDOUBLE_CLASS, mxREAL, MX_GPU_DO_NOT_INITIALIZE);
    double* svtBPtr = (double*)(mxGPUGetData(svtBIn));
    
    if(s[0] != 0){
        combineScaleB<<<gridSize(n), BLOCK_SIZE>>>(stBPtr,svtBPtr,tAPtr,vtAPtr,tBPtr,vtBPtr,RAPtr,vRAPtr,RBPtr,vRBPtr,RPtr,vRPtr,tPtr,n);
        CudaCheckError();
        findErrVar<<<gridSize(n), BLOCK_SIZE>>>(outPtr,tAPtr,vtAPtr,stBPtr,svtBPtr,RAPtr,vRAPtr,RBPtr,vRBPtr,RPtr,vRPtr,tPtr,n);
        CudaCheckError();
    }
    else{
        findErrVar<<<gridSize(n), BLOCK_SIZE>>>(outPtr,tAPtr,vtAPtr,tBPtr,vtBPtr,RAPtr,vRAPtr,RBPtr,vRBPtr,RPtr,vRPtr,tPtr,n);
        CudaCheckError();
    }
    
	mxGPUDestroyGPUArray(RIn);
	mxGPUDestroyGPUArray(vRIn);
	mxGPUDestroyGPUArray(tAIn);
	mxGPUDestroyGPUArray(vtAIn);
	mxGPUDestroyGPUArray(RAIn);
	mxGPUDestroyGPUArray(vRAIn);
	mxGPUDestroyGPUArray(tBIn);
	mxGPUDestroyGPUArray(vtBIn);
	mxGPUDestroyGPUArray(RBIn);
	mxGPUDestroyGPUArray(vRBIn);
	mxGPUDestroyGPUArray(tIn);

    mxGPUDestroyGPUArray(stBIn);
    mxGPUDestroyGPUArray(svtBIn);
	mxGPUDestroyGPUArray(outMat);
}
