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
    int* _d = (int*)mxGetPr(prhs[0]);
    
    int now_need = 4;
    if(allnum < now_need)
        mexErrMsgTxt("����VVP�������ݲ���");
    int edgeNum = *_d;
    _d++;
    if(edgeNum == 0)
    {
        plhs[0] = NULL;
        return;
    }
        
    plhs[0] = mxCreateCellMatrix(edgeNum, 1);
    int edgeiNum;
    for(int i = 0; i < edgeNum; i++)
    {
        now_need += 4;
        if(allnum < now_need)
            mexErrMsgTxt("����VVP�������ݲ���");
        edgeiNum = *_d;
        _d++;
        if(edgeiNum == 0)
            continue;
        now_need += edgeiNum * 2 * 4;
        if(allnum < now_need)
            mexErrMsgTxt("����VVP�������ݲ���");
        
        mxArray* pdata = mxCreateDoubleMatrix(edgeiNum, 2, mxREAL);
        double* _pdata = (double*)mxGetPr(pdata);
        for(int i = 0; i < edgeiNum;i++)
        {
            _pdata[i] = *_d;
            _d++;
            _pdata[i + edgeiNum] = *_d;
            _d++;
        }
        mxSetCell(plhs[0], i, pdata);
    }

}