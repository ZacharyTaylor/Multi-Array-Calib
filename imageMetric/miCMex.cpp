/*MI A mex implementation to quickly find the mutual information of two
 *   inputs. Designed to be called through findMI.m
 *
 *   Required Inputs:
 *   A- 1st input, contains intergers in range 0 to bins-1
 *   B- 2nd input, contains the same number of elements as A, with the
 *   same range limits
 *   normal- true for NMI false for MI
 *   bins- number of bins to use when finding MI
 *
 *   Outputs:
 *   mi- 1x1 mutual information value
 *
 *   References-
 *   This code was used in generating the results for the journal paper
 *   Multi-modal sensor calibration using a gradient orientation measure 
 *   http://www.zjtaylor.com/welcome/download_pdf?pdf=JFR2013.pdf as well
 *   as several of my other publications
 *
 *   This code was written by Zachary Taylor
 *   zacharyjeremytaylor@gmail.com
 *   http://www.zjtaylor.com
 */

#include "mex.h"
#include "matrix.h"
#include <math.h>

#define printf mexPrintf
#define NUM_ARGS 4

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
	//check inputs
    if (nrhs != 4 || (nlhs != 1 && nlhs != 0))
      mexErrMsgIdAndTxt("MI:BadNArgs","Need 4 inputs and 1 output.");
    
    //check input types
    if(mxGetClassID(prhs[0]) != mxSINGLE_CLASS)
        mexErrMsgIdAndTxt("MI:BadArg","A must be of type single");
    if(mxGetClassID(prhs[1]) != mxSINGLE_CLASS)
        mexErrMsgIdAndTxt("MI:BadArg","B must be of type single");
    if(mxGetClassID(prhs[2]) != mxLOGICAL_CLASS)
        mexErrMsgIdAndTxt("MI:BadArg","normal must be of type logical");
    if(mxGetClassID (prhs[3]) != mxUINT32_CLASS)
        mexErrMsgIdAndTxt("bins:BadArg","bins must be of type uint32");
        
	//get sizes
	size_t numEl[NUM_ARGS];
	for(size_t i = 0; i < NUM_ARGS; i++){
		size_t numDim = mxGetNumberOfDimensions(prhs[i]);
		numEl[i] = 1;
		for(size_t j = 0; j < numDim; j++)
			numEl[i] *= mxGetDimensions(prhs[i])[j];
	}

	//check sizes
    if(numEl[0] != numEl[1])
        mexErrMsgIdAndTxt("MI:BadArg","A and B must have the same number of elements");
    if(numEl[2] != 1)
        mexErrMsgIdAndTxt("MI:BadArg","normals must be a single value");
    if(numEl[3] != 1)
        mexErrMsgIdAndTxt("MI:BadArg","bins must be a single value");
           
    
    //get value of input variables
    float *A = (float *) mxGetData(prhs[0]);
    float *B = (float *) mxGetData(prhs[1]);
    mxLogical normal = ((mxLogical *)mxGetPr(prhs[2]))[0];
    uint32_T bins = ((uint32_T *)mxGetPr(prhs[3]))[0];
    
    //setup outputs
    plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
    double *mi = mxGetPr(plhs[0]);
    
    //local variables
    float *hAB = new float[bins*bins];
    float *hA = new float[bins];
    float *hB = new float[bins];
            
    double entB, entA, entAB;
       
    //zero histograms
    for(size_t i = 0; i < bins; i++){
        hB[i] = 0;
        hA[i] = 0;
        
        for(size_t j = 0; j < bins; j++){
            hAB[i + j*bins] = 0;
        }
    }
    
    //get joint histogram
    for(size_t i = 0; i < numEl[0]; i++){
        
		unsigned int x = ((float)(bins-1))*A[i];
		unsigned int y = ((float)(bins-1))*B[i];
		unsigned int idx = x + bins*y;
		
        if((x >= bins) || (y >= bins) || (x < 0) || (y < 0)){
			mexErrMsgIdAndTxt("MI:BadRange","All values of A and B must be between 0 and 1");
		}
        
        hAB[idx]++;
    }
       
    //normalize
    for(size_t i = 0; i < bins*bins; i++){
        hAB[i] = hAB[i] / numEl[0];
    }
    
    //Get individual histograms
    for(size_t i = 0; i < bins; i++){
        for(size_t j = 0; j < bins; j++){
            hA[i] += hAB[j + i*bins];
            hB[i] += hAB[i + j*bins];
        }
    }
          
    //calculate entropy
    entAB = 0; entB = 0; entA = 0;  
    for(size_t i = 0; i < bins; i++){
        
        if(hA[i] != 0){
            entA -= hA[i]*log(hA[i])/log(2.0);
        }
        if(hB[i] != 0){
            entB -= hB[i]*log(hB[i])/log(2.0);
        }
        
        for(size_t j = 0; j < bins; j++){
            if(hAB[i + bins*j] != 0){
                entAB -= hAB[i + bins*j]*log(hAB[i + bins*j])/log(2.0);
            }
        }
    }
    
    //calculate mi
    if(normal){
        mi[0] = (entB + entA)/entAB;
    }
    else{
        mi[0] = entB + entA - entAB;
    }
            
    delete hAB; delete hA; delete hB;
}