#include <tensorRT/preprocess_kernel.cuh>
#include <tensorRT/trt_tensor.hpp>
#include <tensorRT/trt_builder.hpp>
#include <tensorRT/trt_infer.hpp>
#include <tensorRT/ilogger.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <thread>
#include <dirent.h>
#include <numeric>

#include <cuda_runtime.h>
#include <NvInfer.h>
#include <NvCaffeParser.h>
#include <NvInferPlugin.h>

using namespace std;

const std::vector<float> img_means = {12.12, 10.88, 0.23, -1.04, 0.21};
const std::vector<float> img_stds = {12.32, 11.47, 6.91, 0.86, 0.16};

std::vector<std::string> ReadAllImages(const std::string& img_dir_path)
{
  std::vector<std::string> filenames;
  // sweep the directory
  DIR* dir;
  if ((dir = opendir(img_dir_path.c_str())) == nullptr) {
    throw std::runtime_error("directory " + img_dir_path + " does not exist");
  }
  dirent* dp;
  for (dp = readdir(dir); dp != nullptr; dp = readdir(dir)) {
    const std::string img_file_name = dp->d_name;
    if (img_file_name == "." || img_file_name == "..") {
      continue;
    }
    filenames.push_back(img_dir_path + "/" + img_file_name);
  }
  closedir(dir);

  std::sort(filenames.begin(), filenames.end());
  return filenames;
}

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v. >: decrease <: increase
  std::sort(idx.begin(), idx.end(),
        [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

  return idx;
}

struct AffineMatrix{
    float i2d[6];       // image to dst(network), 2x3 matrix
    float d2i[6];       // dst to image, 2x3 matrix

    void compute(const cv::Size& from, const cv::Size& to, bool fix=true){
        float scale_x = to.width / (float)from.width;
        float scale_y = to.height / (float)from.height;
        float scale = std::min(scale_x, scale_y);
        /* 
                + scale * 0.5 - 0.5 的主要原因是使得中心更加对齐，下采样不明显，但是上采样时就比较明显
            参考：https://www.iteye.com/blog/handspeaker-1545126
        */
        if(fix) {
          i2d[0] = scale;  i2d[1] = 0;  i2d[2] = -scale * from.width  * 0.5  + to.width * 0.5 + scale * 0.5 - 0.5;
          i2d[3] = 0;  i2d[4] = scale;  i2d[5] = -scale * from.height * 0.5 + to.height * 0.5 + scale * 0.5 - 0.5;
        }
        else {
          i2d[0] = scale_x;  i2d[1] = 0;  i2d[2] = -scale_x * from.width  * 0.5  + to.width * 0.5 + scale_x * 0.5 - 0.5;
          i2d[3] = 0;  i2d[4] = scale_y;  i2d[5] = -scale_y * from.height * 0.5 + to.height * 0.5 + scale_y * 0.5 - 0.5;
        }
        
        cv::Mat m2x3_i2d(2, 3, CV_32F, i2d);
        cv::Mat m2x3_d2i(2, 3, CV_32F, d2i);
        cv::invertAffineTransform(m2x3_i2d, m2x3_d2i);
    }

    cv::Mat i2d_mat(){
        return cv::Mat(2, 3, CV_32F, i2d);
    }

    cv::Mat d2i_mat(){
        return cv::Mat(2, 3, CV_32F, d2i);
    }
};

static cv::Mat warpAffine(const cv::Mat& image, shared_ptr<TRT::Tensor>& tensor, int ibatch){

  CUDAKernel::Norm normalize = CUDAKernel::Norm::alpha_beta(1.0, 0.0, CUDAKernel::ChannelType::None);
  cv::Size input_size(tensor->size(3), tensor->size(2));
  printf("Affine Transform [%dx%d]-->[%dx%d]\n", image.rows, image.cols, tensor->size(2), tensor->size(3));
  AffineMatrix affine;
  affine.compute(image.size(), input_size, true);
  std::cout << affine.i2d_mat() << std::endl;

  size_t size_image      = image.cols * image.rows * 5;
  size_t size_matrix     = iLogger::upbound(sizeof(affine.d2i), 32);
  auto workspace         = tensor->get_workspace();
  uint8_t* gpu_workspace        = (uint8_t*)workspace->gpu(size_matrix + size_image);
  float*   affine_matrix_device = (float*)gpu_workspace;
  uint8_t* image_device         = size_matrix + gpu_workspace;

  uint8_t* cpu_workspace        = (uint8_t*)workspace->cpu(size_matrix + size_image);
  float* affine_matrix_host     = (float*)cpu_workspace;
  uint8_t* image_host           = size_matrix + cpu_workspace;
  auto stream                   = tensor->get_stream();

  memcpy(image_host, image.data, size_image);
  memcpy(affine_matrix_host, affine.d2i, sizeof(affine.d2i));
  checkCudaRuntime(cudaMemcpyAsync(image_device, image_host, size_image, cudaMemcpyHostToDevice, stream));
  checkCudaRuntime(cudaMemcpyAsync(affine_matrix_device, affine_matrix_host, sizeof(affine.d2i), cudaMemcpyHostToDevice, stream));

  CUDAKernel::warp_affine_bilinear_and_normalize_plane(
    image_device,               image.cols * 5,       image.cols,       image.rows, 
    tensor->gpu<float>(ibatch), input_size.width,     input_size.height, 
    affine_matrix_device, 128, 
    normalize, stream
  );
  return affine.d2i_mat().clone();
}

cv::Mat ProjectSphere(const std::vector<float>& scan, const uint32_t& num_points,
                                              const float _fov_up, const float _fov_down,
                                              const int _img_h, const int _img_w) {
      
  typedef cv::Vec<float, 5> Vec5f;
  
  float fov_up = _fov_up / 180.0 * M_PI;    // field of view up in radians
  float fov_down = _fov_down / 180.0 * M_PI;  // field of view down in radians
  float fov = std::abs(fov_down) + std::abs(fov_up); // get field of view total in radians

  std::vector<float> ranges;
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  std::vector<float> intensitys;

  std::vector<float> proj_xs_tmp;
  std::vector<float> proj_ys_tmp;

  cv::Mat sphere_img(_img_h, _img_w, CV_32FC(5), cv::Vec<float, 5>(0, 0, 0, 0, 0));

  for (uint32_t i = 0; i < num_points; i++) {
    float x = scan[4 * i];
    float y = scan[4 * i + 1];
    float z = scan[4 * i + 2];
    float intensity = scan[4 * i + 3];
    float range = std::sqrt(x*x+y*y+z*z);
    ranges.push_back(range);
    xs.push_back(x);
    ys.push_back(y);
    zs.push_back(z);
    intensitys.push_back(intensity);

    // get angles
    float yaw = -std::atan2(y, x);
    float pitch = std::asin(z / range);

    // get projections in image coords
    float proj_x = 0.5 * (yaw / M_PI + 1.0); // in [0.0, 1.0]
    float proj_y = 1.0 - (pitch + std::abs(fov_down)) / fov; // in [0.0, 1.0]

    // scale to image size using angular resolution
    proj_x *= _img_w; // in [0.0, W]
    proj_y *= _img_h; // in [0.0, H]

    // round and clamp for use as index
    proj_x = std::floor(proj_x);
    proj_x = std::min(_img_w - 1.0f, proj_x);
    proj_x = std::max(0.0f, proj_x); // in [0,W-1]
    // proj_xs_tmp.push_back(proj_x);

    proj_y = std::floor(proj_y);
    proj_y = std::min(_img_h - 1.0f, proj_y);
    proj_y = std::max(0.0f, proj_y); // in [0,H-1]
    // proj_ys_tmp.push_back(proj_y);

    sphere_img.at<Vec5f>(proj_y, proj_x) = cv::Vec<float, 5>(range, x, y, z, intensity);
  }
  return sphere_img;
}

using namespace nvinfer1;
class Logger : public ILogger {
public:
	virtual void log(Severity severity, const char* msg) noexcept override {

		if (severity == Severity::kINTERNAL_ERROR) {
			INFOE("NVInfer INTERNAL_ERROR: %s", msg);
			abort();
		}else if (severity == Severity::kERROR) {
			INFOE("NVInfer: %s", msg);
		}
		else  if (severity == Severity::kWARNING) {
			INFOW("NVInfer: %s", msg);
		}
		else  if (severity == Severity::kINFO) {
			INFOD("NVInfer: %s", msg);
		}
		else {
			INFOD("%s", msg);
		}
	}
};

int main(int argc, char **argv){


  cudaSetDevice(0);
  // create a model using the API directly and serialize it to a stream
  char *trtModelStream{nullptr};
  size_t size{0};

  if (argc == 4 && std::string(argv[2]) == "-i") {
      const std::string engine_file_path {argv[1]};
      std::ifstream file(engine_file_path, std::ios::binary);
      if (file.good()) {
          file.seekg(0, file.end);
          size = file.tellg();
          file.seekg(0, file.beg);
          trtModelStream = new char[size];
          assert(trtModelStream);
          file.read(trtModelStream, size);
          file.close();
      }
  } else {
      std::cerr << "arguments not right!" << std::endl;
      std::cerr << "run 'python3 yolox/deploy/trt.py -n yolox-{tiny, s, m, l, x}' to serialize model first!" << std::endl;
      std::cerr << "Then use the following command:" << std::endl;
      std::cerr << "./yolox ../model_trt.engine -i ../../../assets/dog.jpg  // deserialize file and run inference" << std::endl;
      return -1;
  }
  const std::string input_image_path {argv[3]};

  //std::vector<std::string> file_names;
  //if (read_files_in_dir(argv[2], file_names) < 0) {
      //std::cout << "read_files_in_dir failed." << std::endl;
      //return -1;
  //}
  static Logger gLogger;
  nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
  assert(runtime != nullptr);
  nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
  assert(engine != nullptr); 
  nvinfer1::IExecutionContext* context = engine->createExecutionContext();



  // int deviceid = 0;
  // TRT::set_device(deviceid);

  // if(!iLogger::exists("rangenet.fp32.trtmodel")){
  //   printf("rangenet.fp32.trtmodel doesn't exist! Build the engine!\n");
  //   TRT::compile(TRT::Mode::FP32, 1, "../darknet53/model.onnx", "rangenet.fp32.trtmodel");
  // }

  // std::shared_ptr<TRT::Infer> segmentor = TRT::load_infer("rangenet.fp32.trtmodel");
  // if(segmentor == nullptr) {
  //   INFOE("Engine is nullptr");
  //   return -1;
  // }

  // std::shared_ptr<TRT::Tensor> input = segmentor->tensor("x.1");
  // std::shared_ptr<TRT::Tensor> output = segmentor->tensor("647");  

  // // Open a scan
  // std::string scan_file = "example.bin";
  // std::ifstream in(scan_file.c_str(), std::ios::binary);
  // if (!in.is_open()) {
  //   std::cerr << "Could not open the scan!" << std::endl;
  //   return 1;
  // }

  // in.seekg(0, std::ios::end);
  // uint32_t num_points = in.tellg() / (4 * sizeof(float));
  // in.seekg(0, std::ios::beg);

  // std::vector<float> values(4 * num_points);
  // in.read((char*)&values[0], 4 * num_points * sizeof(float));
  // // std::vector<std::vector<float>> input_img = ProjectSphere(values, num_points, 3, -25, 64, 2048);

  // // bool all_zeros = false;
  // // std::vector<int> invalid_idxs;

  // // for (uint32_t pixel_id = 0; pixel_id < input_img.size(); pixel_id++){
  // //   // check if the pixel is invalid
  // //   all_zeros = std::all_of(input_img[pixel_id].begin(), input_img[pixel_id].end(), [](int i) { return i==0.0f; });
  // //   if (all_zeros) {
  // //     invalid_idxs.push_back(pixel_id);
  // //   }
  // //   for (int i = 0; i < 5; i++) {
  // //     // normalize the data
  // //     if (!all_zeros) {
  // //       input_img[pixel_id][i] = (input_img[pixel_id][i] - img_means[i]) / img_stds[i];
  // //     }
  // //   }
  // // }

  // cv::Mat input_mat = ProjectSphere(values, num_points, 3, -25, 64, 2048);
  // int max_batch_size = 1;
  // input->resize_single_dim(0, max_batch_size).to_gpu();  

  // int ibatch = 0;
  // cv::Mat imat_left = warpAffine(input_mat, input, ibatch);
  // segmentor->forward(true);

  

  return 0;
}