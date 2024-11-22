#include "mex.h"
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
    
    size_t allnum = mxGetNumberOfElements(prhs[0]);
    char* _d = (char*)mxGetPr(prhs[0]);
    
    int now_need = 4;
    if(allnum < now_need)
        mexErrMsgTxt("����VVP�������ݲ���");
    int fsaNum = *((int*)_d);
    _d+=4;
    now_need += fsaNum * fsaNum;
    if(allnum < now_need)
        mexErrMsgTxt("����VVP�������ݲ���");
    plhs[0] = mxCreateDoubleMatrix(fsaNum, fsaNum, mxREAL);
    double* _pdata = (double*)mxGetPr(plhs[0]);
    
    for(int i =0; i < fsaNum; i++)
    {
        for(int j = 0; j < fsaNum; j++)
        {
            _pdata[j * fsaNum + i] = *_d;
            _d++;
        }
    }
    
}