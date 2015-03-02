/*COLOURIMAGE A mex implementation to quickly colour an image given a set
 *of 2D Points. Designed to be called through points2Image.m
 *
 *   Required Inputs:
 *   locs- nx2 set of 2d points, (x,y)
 *   colours- nxm set of intensity values
 *   disk- rxr array giving opacity of sprite to assign to each point loc
 *   imageSize- 1x2 vector giving the height and width of the output image 
 *   [h,w]
 *
 *   Outputs:
 *   image- hxwxm output image
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

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    
    //check inputs
    if (nrhs != 5 || (nlhs != 1 && nlhs != 0))
      mexErrMsgIdAndTxt("COLOURIMAGE:BadNArgs","Need 4 inputs and 1 output.");
    
    //check input types
    if(mxGetClassID(prhs[0]) != mxDOUBLE_CLASS)
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","locs must be of type double");
    if(mxGetClassID(prhs[1]) != mxDOUBLE_CLASS)
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","colours must be of type double");
    if(mxGetClassID(prhs[2]) != mxDOUBLE_CLASS)
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","disk must be of type double");
    if(mxGetClassID (prhs[3]) != mxUINT32_CLASS)
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","imageSize must be of type uint32");
        
    //check number of dimensions
    for(int i = 0; i < 4; i++){
        if(mxGetNumberOfDimensions(prhs[i]) != 2)
            mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","All arguments must be 2D");
    }
    
    //check sizes
    if(mxGetDimensions(prhs[0])[1] != 2)
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","locs must have 2 columns [x,y]");
    if(mxGetDimensions(prhs[0])[0] != mxGetDimensions(prhs[1])[0])
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","locs and colours must be the same length");
    if((mxGetDimensions(prhs[2])[0] != mxGetDimensions(prhs[2])[1]))
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","disk must be a square matrix");
    if((mxGetDimensions(prhs[3])[0] != 1) || (mxGetDimensions(prhs[3])[1]) != 2)
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","imageSize must be 1x2");
        
    //get value of remaining input variables
    double * locs = mxGetPr(prhs[0]);
    double * colours = mxGetPr(prhs[1]);
    double * disk = mxGetPr(prhs[2]);
    double * baseImage = mxGetPr(prhs[4]);
    
    //get required variable sizes
    size_t length = mxGetDimensions(prhs[0])[0];
    size_t imageHeight = ((uint32_T *) mxGetData(prhs[3]))[0];
    size_t imageWidth = ((uint32_T *) mxGetData(prhs[3]))[1];
    size_t imageDepth = mxGetDimensions(prhs[1])[1];
    int pointRadius = (mxGetDimensions(prhs[2])[0]-1)/2;
    
    //check size
    if((imageHeight == 0) || (imageWidth == 0))
        mexErrMsgIdAndTxt("COLOURIMAGE:BadArg","imageSize cannot be zero");
       
    //setup output
    const mwSize outSize[] = {imageHeight, imageWidth, imageDepth};
    plhs[0] = mxCreateNumericArray(3, outSize, mxDOUBLE_CLASS, mxREAL);
    double *image = mxGetPr(plhs[0]);
    
    //colour image
    for(int i = 0; i < length; i++){
        for(int iy = -pointRadius; iy < pointRadius; iy++){
            for(int ix = -pointRadius; ix < pointRadius; ix++){
                //check if point inside image
                bool valid = (locs[i]+ix >= 0) && (locs[i]+ix < imageWidth) && (locs[i+length]+iy >= 0) && (locs[i+length]+iy < imageHeight);
                
                if(valid){
                    for(int id = 0; id < imageDepth; id++){
                        //get opacity of point
                        double opac = disk[iy+pointRadius + (2*pointRadius+1)*(ix+pointRadius)];
                        //get index of point
                        int idx = locs[i+length]+iy + (locs[i]+ix)*imageHeight + id*imageHeight*imageWidth;
                        //add point to image
                        baseImage[idx] = (1-opac)*baseImage[idx] + opac*colours[i + length*id];
                    }
                }
            }
        }
    }
    
    for(int i = 0; i < imageHeight*imageWidth*imageDepth; i++){
        image[i] = baseImage[i];
    }
}