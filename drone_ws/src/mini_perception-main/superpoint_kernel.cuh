#ifndef SUPERPOINT_KERNEL_H
#define SUPERPOINT_KERNEL_h


#include "cuda_tools.hpp"

void nms_kernel(const float *data, int cell, int h, int w, int cell_size, float3 *kps);

void normalize_descriptor_kernel(float *data);

#endif