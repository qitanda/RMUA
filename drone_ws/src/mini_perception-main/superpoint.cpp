#include <trt_builder.hpp>
#include <trt_infer.hpp>
#include <ilogger.hpp>
#include <lanedet/lanedet.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <thread>
#include <dirent.h>
#include <eigen3/Eigen/Core>

using namespace std;

void CellNMS(const cv::Mat& prob, std::vector<cv::Point2f>& kps)
{
  // std::cout << kpts.scalar_type() << sizeof(kpts.scalar_type()) << std::endl;
  // NMS alternative
  int rows = prob.rows;
  int cols = prob.cols;

  int cell_w = 20, cell_h = 20;
  int cell_x_size = cols/cell_w;
  int cell_y_size = rows/cell_h;

  for(int cell_row_id = 0; cell_row_id < cell_y_size; ++cell_row_id) {
    for(int cell_col_id = 0; cell_col_id < cell_x_size; ++cell_col_id) {
      int row_offset = cell_row_id * cell_h;
      int col_offset = cell_col_id * cell_w;
      float max_prob = 0;
      int max_col_id = -1;
      int max_row_id = -1;
      for(int px_row_id = row_offset; px_row_id < row_offset + cell_h; px_row_id ++) {
        for(int px_col_id = col_offset; px_col_id < col_offset + cell_w; px_col_id ++) {
          if(prob.ptr<float>(px_row_id)[px_col_id] > max_prob){
            max_prob = prob.ptr<float>(px_row_id)[px_col_id];
            max_col_id = px_col_id;
            max_row_id = px_row_id;
          }
        }
      }
      if( max_col_id != -1 && max_row_id != -1 && max_prob > 0.1) {
        kps.push_back(cv::Point2f(max_col_id, max_row_id));
      }
    }
  }
}

// void nms(const TRT::Tensor& prob, std::vector<cv::Point2f>& kps)
// {
//   auto grid  = CUDATools::grid_dims(jobs);
//   auto block = CUDATools::block_dims(jobs);
// }

static inline bool inBorder(int64_t h, int64_t w, int64_t H, int64_t W) 
{
  return h >= 0 && h < H && w >= 0 && w < W;
}

void extractDescriptor(float *desc_ptr, Eigen::MatrixXf& desc_mat, std::vector<cv::Point2f>& kps, int kp_size)
{
  int channel = 256;
	int height = 60;
	int width = 80;

	float * desc_channel_sum_sqrts = new float[height*width];
	for (int dfh = 0; dfh < height; dfh++) {
		for (int dfw = 0; dfw < width; dfw++) {
			float desc_channel_sum_temp = 0.f;
			for (int dfc = 0; dfc < channel; dfc++) {
				desc_channel_sum_temp += desc_ptr[dfc*width*height + dfh*width + dfw]*
					desc_ptr[dfc*width*height + dfh*width + dfw];
			}
			float desc_channel_sum_sqrt = std::sqrt(desc_channel_sum_temp);
			desc_channel_sum_sqrts[dfh*width + dfw] = desc_channel_sum_sqrt;
		}
	}
	
	for (int dfh = 0; dfh < height; dfh++) {
		for (int dfw = 0; dfw < width; dfw++) {
			for (int dfc = 0; dfc < channel; dfc++) {
				desc_ptr[dfc*width*height + dfh*width + dfw] = desc_ptr[dfc*width*height + dfh*width + dfw] / desc_channel_sum_sqrts[dfh*width + dfw];
			}
		}
	}
	int s = 8;
  desc_mat.resize(channel, kp_size);

	for (int pid = 0; pid < kp_size; ++pid) {
    auto &kp = kps[pid];
		float ix = (kp.x - s / 2 + 0.5) / (width*s - s / 2 - 0.5) * (width - 1);
		float iy = (kp.y - s / 2 + 0.5) / (height*s - s / 2 - 0.5) * (height - 1);

		int ix_nw = std::floor(ix);
		int iy_nw = std::floor(iy);

		int ix_ne = ix_nw + 1;
		int iy_ne = iy_nw;

		int ix_sw = ix_nw;
		int iy_sw = iy_nw + 1;

		int ix_se = ix_nw + 1;
		int iy_se = iy_nw + 1;

		float nw = (ix_se - ix) * (iy_se - iy);
		float ne = (ix - ix_sw) * (iy_sw - iy);
		float sw = (ix_ne - ix) * (iy - iy_ne);
		float se = (ix - ix_nw) * (iy - iy_nw);

		float descriptors_channel_sum_l2 = 0.f;
		for (int dfc = 0; dfc < channel; dfc++) {
			float res = 0.f;

			if (inBorder(iy_nw, ix_nw, height, width)) {
				res += desc_ptr[dfc*height*width + iy_nw*width + ix_nw] * nw;
			}
			if (inBorder(iy_ne, ix_ne, height, width)) {
				res += desc_ptr[dfc*height*width + iy_ne*width + ix_ne] * ne;
			}
			if (inBorder(iy_sw, ix_sw, height, width)) {
				res += desc_ptr[dfc*height*width + iy_sw*width + ix_sw] * sw;
			}
			if (inBorder(iy_se, ix_se, height, width)) {
				res += desc_ptr[dfc*height*width + iy_se*width + ix_se] * se;
			}
      desc_mat(dfc, pid) = res;
		}
	}

  for(int pid = 0; pid < kp_size; ++pid) {
    desc_mat.col(pid).normalize();
  }
}


std::vector<std::pair<int, int>> TransferMat(const Eigen::MatrixXf mat, const double &nn_threshod){
  std::vector<std::pair<int, int>> matches;
  const int mat_row = mat.rows();
  const int mat_col = mat.cols();

  for(int i = 0; i < mat_row; i++){
      double cur_row_max_value = -1;
      int cur_row_max_idx = -1;
      for(int j = 0;  j < mat_col; j++){
          if(mat(i, j) > cur_row_max_value){
              cur_row_max_value = mat(i, j);
              cur_row_max_idx = j;
          }
      }
      if(cur_row_max_value > nn_threshod && cur_row_max_idx >= 0){
          matches.push_back({i, cur_row_max_idx});
      }
  }
  return matches;
}

int main(){

  int deviceid = 0;
  TRT::set_device(deviceid);

  if(!iLogger::exists("superpoint.fp32.trtmodel")){
    printf("superpoint.fp32.trtmodel doesn't exist! Build the engine!\n");
    TRT::compile(TRT::Mode::FP32, 1, "superpoint.onnx", "superpoint.fp32.trtmodel");
  }

  std::shared_ptr<TRT::Infer> superpoint = TRT::load_infer("superpoint.fp32.trtmodel");
  if(superpoint == nullptr) {
    INFOE("Engine is nullptr");
    return -1;
  }

  cv::Mat img = cv::imread("./demo.png", cv::IMREAD_GRAYSCALE);
  std::shared_ptr<TRT::Tensor> input = superpoint->tensor("image");
  std::shared_ptr<TRT::Tensor> semi = superpoint->tensor("semi");  
  std::shared_ptr<TRT::Tensor> desc = superpoint->tensor("desc");  

  int eval_w = input->size(3), eval_h = input->size(2);

  int max_batch_size = 1;
  input->resize_single_dim(0, max_batch_size).to_gpu();  

  int ibatch = 0;
  INFO("input.shape = %s", input->shape_string());
  INFO("semi.shape = %s", semi->shape_string());

  img.convertTo(img, CV_32F, 1/255.0f, 0.0f);
  int size_image = img.rows * img.cols * sizeof(float);
  int size_desc = 4 * img.rows * img.cols * sizeof(float);
  memcpy(input->cpu<float>(), img.data, size_image);
  checkCudaRuntime(cudaMemcpyAsync(input->gpu<float>(), input->cpu<float>(), size_image, cudaMemcpyHostToDevice, input->get_stream()));

  superpoint->forward(true);
  superpoint->forward(true);
  superpoint->forward(true);
  superpoint->forward(true);
  superpoint->forward(true);


  double start = cv::getTickCount();
  superpoint->forward(true);

  float *prob_ptr = semi->cpu<float>(ibatch);
  float *desc_ptr = desc->cpu<float>(ibatch);
  cv::Mat prob_img(img.rows, img.cols, CV_32F, cv::Scalar(0));
  cv::Mat desc_img(img.rows/8, img.cols/8, CV_32FC(256), cv::Scalar(0));
  memcpy((float*)prob_img.data, prob_ptr, size_image);
  // memcpy((float*)desc_img.data, desc_ptr, size_desc);

  std::vector<cv::Point2f> kps;
  CellNMS(prob_img, kps);
  double end = cv::getTickCount();

  // float * desc_output_f;
  Eigen::MatrixXf desc_mat0;
  extractDescriptor(desc->cpu<float>(ibatch), desc_mat0, kps, kps.size());
  std::cout << "dt: " << (end - start) / cv::getTickFrequency() << std::endl;
  cv::Mat show_img;
  cv::cvtColor(img, show_img, cv::COLOR_GRAY2BGR);
  for(auto p: kps) {
    cv::circle(show_img, p, 2, cv::Scalar(0, 0, 255), -1);
  }


  cv::Mat imgR = cv::imread("./demo2.png", cv::IMREAD_GRAYSCALE);
  imgR.convertTo(imgR, CV_32F, 1/255.0f, 0.0f);
  memcpy(input->cpu<float>(), imgR.data, size_image);
  checkCudaRuntime(cudaMemcpyAsync(input->gpu<float>(), input->cpu<float>(), size_image, cudaMemcpyHostToDevice, input->get_stream()));
  superpoint->forward(true);
  prob_ptr = semi->cpu<float>(ibatch);
  desc_ptr = desc->cpu<float>(ibatch);
  cv::Mat prob_img2(img.rows, img.cols, CV_32F, cv::Scalar(0));
  cv::Mat desc_img2(img.rows/8, img.cols/8, CV_32FC(256), cv::Scalar(0));
  memcpy((float*)prob_img2.data, prob_ptr, size_image);
  // memcpy((float*)desc_img2.data, desc_ptr, size_desc);
  std::vector<cv::Point2f> kps2;
  CellNMS(prob_img2, kps2);
  Eigen::MatrixXf desc_mat1;
  extractDescriptor(desc->cpu<float>(ibatch), desc_mat1, kps2, kps2.size());
  

  cv::Mat show_img2;
  cv::cvtColor(imgR, show_img2, cv::COLOR_GRAY2BGR);
  for(auto p: kps2) {
    cv::circle(show_img2, p, 2, cv::Scalar(0, 0, 255), -1);
  }

  Eigen::MatrixXf affinity = desc_mat0.transpose() * desc_mat1;
  auto matches = TransferMat(affinity, 0.4);
  
  cv::Mat hcon_img;
  cv::hconcat(show_img, show_img2, hcon_img);

  for(auto match: matches) {
    auto p1 = kps[match.first];
    auto p2 = kps2[match.second];
    cv::line(hcon_img, p1, cv::Point2f(p2.x + 640, p2.y), cv::Scalar(255, 0, 0), 1);
  }
  // cv::Mat kpt_img(img.rows, img.cols, CV_8U, cv::Scalar(0));
  // kpt_img.setTo(1, prob_img > 0.1);
  
  // cv::Mat show_img;
  // cv::cvtColor(img, show_img, cv::COLOR_GRAY2BGR);
  // for(int i = 0; i < img.rows; ++i) {
  //   for(int j = 0; j < img.cols; ++j) {
  //     if(kpt_img.at<uchar>(i, j) == 1) {
  //       cv::circle(show_img, cv::Point(j, i), 2, cv::Scalar(0, 0, 255), -1);
  //     }
  //   }
  // }

  cv::imshow("show", hcon_img);
  // cv::imshow("show2", show_img2);
  cv::waitKey(0);
  return 0;
}