#include "mex.h"
#include "MeasureTools.h"


#include "iostream"
using namespace std;


void mexFunction(int nlhs,       mxArray *plhs[],
		 int nrhs, const mxArray *prhs[])        
{
    if(nlhs!=1)
    {
        mexErrMsgTxt("���������������Ϊ1��.");
    }
    if(nrhs!=1)
    {
        mexErrMsgTxt("���������������Ϊ1��.");
    }
    
    // Get the data;
    double *elpshape = (double *)mxGetPr(prhs[0]);
    
    // Outputdata
    //mexPrintf("%d, %d\n", ng, nf);
    plhs[0] = mxCreateDoubleMatrix(1,6,mxREAL);
    
    double *parm = (double *)mxGetPr(plhs[0]);
    
    ELPShape2Equation(elpshape, parm);
    
}

