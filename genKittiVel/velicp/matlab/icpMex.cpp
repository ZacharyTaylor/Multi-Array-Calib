#include "mex.h"
#include <iostream>
#include <string.h>

#include "icpPointToPlane.h"
#include "icpPointToPoint.h"

using namespace std;

void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

  // check arguments
  if (nrhs!=6)
    mexErrMsgTxt("6 input parameters required (M,T,Tr,t,inlier distance,method=point_to_point,point_to_plane).");
  if (nlhs!=1)
    mexErrMsgTxt("1 output parameter required (Tr).");
  if (!mxIsDouble(prhs[0]) || mxGetM(prhs[0])!=3)
    mexErrMsgTxt("Input M (model points) must be a double 3xN matrix.");
  if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1])!=3)
    mexErrMsgTxt("Input T (template points) must be a double 3xN matrix.");  
  if (mxGetM(prhs[0])!=mxGetM(prhs[1]))
    mexErrMsgTxt("Input M and T must have same number of rows (=dimensionality).");  
  if (mxGetN(prhs[1])!=mxGetN(prhs[3]))
    mexErrMsgTxt("Input T and t must have same number of columns.");  
  if (!mxIsDouble(prhs[4]) || mxGetM(prhs[4])*mxGetM(prhs[4])!=1)
    mexErrMsgTxt("Input indist (inlier distance) must be a double scalar.");
  
  // read method
  char method[128];
  mxGetString(prhs[5],method,128);
  
  // check method
  if (strcmp(method,"point_to_point") && strcmp(method,"point_to_plane"))
    mexErrMsgTxt("Input method must be either 'point_to_point' or 'point_to_plane'.");

  // input
  double *M         =   (double*)mxGetPr(prhs[0]);
  int32_t M_num     =             mxGetN(prhs[0]);
  double *T         =   (double*)mxGetPr(prhs[1]);
  int32_t T_num     =             mxGetN(prhs[1]);
  int32_t dim       =             mxGetM(prhs[0]);
  double *Tr_in_mex =   (double*)mxGetPr(prhs[2]);
  double *time      =   (double*)mxGetPr(prhs[3]);
  double  indist    = *((double*)mxGetPr(prhs[4]));
  
  // check input transformation dimensionality
  if (dim==3 && (!mxIsDouble(prhs[2]) || mxGetM(prhs[2])!=4 || mxGetN(prhs[2])!=4))
    mexErrMsgTxt("Input Tr must be a double 4x4 matrix for dimensionality 3.");
  
  // input initial transformation
  Matrix R,t;
  R = Matrix(3,3);
  t = Matrix(3,1);
  for (int32_t i=0; i<3; i++) {
    for (int32_t j=0; j<3; j++)
      R.val[i][j] = Tr_in_mex[j*4+i];
    t.val[i][0] = Tr_in_mex[3*4+i];
  }
  
  // run icp
  if (!strcmp(method,"point_to_point")) {
    IcpPointToPoint icp(M,M_num,dim);
    icp.fit(T,time,T_num,R,t,indist);
  } else {
    IcpPointToPlane icp(M,M_num,dim);
    icp.fit(T,time,T_num,R,t,indist);
  }
  
  // output final transformation   
  const int dims[] = {4,4};
  plhs[0]          = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL);
  double *Tr_mex   = (double*)mxGetPr(plhs[0]);
  for (int32_t i=0; i<3; i++) {
    for (int32_t j=0; j<3; j++)
      Tr_mex[j*4+i] = R.val[i][j];
    Tr_mex[3*4+i] = t.val[i][0];
  }
  Tr_mex[15] = 1;

}
