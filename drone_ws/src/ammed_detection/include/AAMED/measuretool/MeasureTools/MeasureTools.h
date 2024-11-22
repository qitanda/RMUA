#pragma once
#include <vector>


//��Բ��״����ת��Ϊһ�㷽�̲���
void ELPShape2Equation(double *elpshape, double *outparms);

// ������Բһ�㷽�̵�yֵ��֪ʱ����������������㣬�����������Ϊ�ռ�
void CalculateRangeAtY(double *elpparm, double y, double *x1, double *x2);



// ������Բ���غ϶�
void CalculateOverlap(double *elp1, double *elp2, double *ration, std::vector<double> *overlapdot);

// ����һ����Բ��������y��ȡֵ��Χ
void CalculateRangeOfY(double *elpshape, double *x_min, double *x_max, double *y_min, double *y_max);

// �����ٵļ�����Բoverlap��һ�ַ���
void fasterCalculateOverlap(double *elp1, double *elp2, double *ration, std::vector<double> *overlapdot);