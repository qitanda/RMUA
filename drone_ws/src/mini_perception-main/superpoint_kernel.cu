#include "superpoint_kernel.cuh"

void nms_kernel(const float *data, int cell, int h, int w, int cell_size, float3 *kps)
{
  int xid = blockDim.x * blockIdx.x + threadIdx.x;
  int yid = blockDim.y * blockIdx.y + threadIdx.y;
  int x = (xid) * cell;
  int y = (yid) * cell;

  float max_prob = 0.0;
  int max_xi = -1, max_yi = -1;
  for(int xi = x; xi < x + cell; xi++) {
    for(int yi = y; yi < y + cell; yi++) {
      if(data[xi + yi * w] > max_prob) {
        max_prob = data[xi + yi * w];
        max_xi = xi;
        max_yi = yi;
      } 
    }
  }
  kps[xid + yid * cell_size] = make_float3(max_xi, max_yi, max_prob);
}

void normalize_descriptor_kernel(float *data)
{

}