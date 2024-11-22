#include <trt_builder.hpp>
#include <trt_infer.hpp>
#include <ilogger.hpp>
#include <yolo/yolo.hpp>
#include <unet/unet.hpp>
#include <stereo/stereo.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <algorithm>
#include <thread>
#include <dirent.h>

using namespace std;

typedef tuple<cv::Mat, cv::Mat, cv::Mat> segment_pair;


// labels_info = [
//     {"unlabeled", "ignoreInEval": True, "id": 0, "color": [0, 0, 0], "trainId": 255},
//     {"ego vehicle", "ignoreInEval": True, "id": 1, "color": [0, 0, 0], "trainId": 255},
//     {"rectification border", "ignoreInEval": True, "id": 2, "color": [0, 0, 0], "trainId": 255},
//     {"out of roi", "ignoreInEval": True, "id": 3, "color": [0, 0, 0], "trainId": 255},
//     {"static", "ignoreInEval": True, "id": 4, "color": [0, 0, 0], "trainId": 255},
//     {"dynamic", "ignoreInEval": True, "id": 5, "color": [111, 74, 0], "trainId": 255},
//     {"ground", "ignoreInEval": True, "id": 6, "color": [81, 0, 81], "trainId": 255},
//     {"road", "ignoreInEval": False, "id": 7, "color": [128, 64, 128], "trainId": 0},
//     {"sidewalk", "ignoreInEval": False, "id": 8, "color": [244, 35, 232], "trainId": 1},
//     {"parking", "ignoreInEval": True, "id": 9, "color": [250, 170, 160], "trainId": 255},
//     {"rail track", "ignoreInEval": True, "id": 10, "color": [230, 150, 140], "trainId": 255},
//     {"building", "ignoreInEval": False, "id": 11, "color": [70, 70, 70], "trainId": 2},
//     {"wall", "ignoreInEval": False, "id": 12, "color": [102, 102, 156], "trainId": 3},
//     {"fence", "ignoreInEval": False, "id": 13, "color": [190, 153, 153], "trainId": 4},
//     {"guard rail", "ignoreInEval": True, "id": 14, "color": [180, 165, 180], "trainId": 255},
//     {"bridge", "ignoreInEval": True, "id": 15, "color": [150, 100, 100], "trainId": 255},
//     {"tunnel", "ignoreInEval": True, "id": 16, "color": [150, 120, 90], "trainId": 255},
//     {"pole", "ignoreInEval": False, "id": 17, "color": [153, 153, 153], "trainId": 5},
//     {"polegroup", "ignoreInEval": True, "id": 18, "color": [153, 153, 153], "trainId": 255},
//     {"traffic light", "ignoreInEval": False, "id": 19, "color": [250, 170, 30], "trainId": 6},
//     {"traffic sign", "ignoreInEval": False, "id": 20, "color": [220, 220, 0], "trainId": 7},
//     {"vegetation", "ignoreInEval": False, "id": 21, "color": [107, 142, 35], "trainId": 8},
//     {"terrain", "ignoreInEval": False, "id": 22, "color": [152, 251, 152], "trainId": 9},
//     {"sky", "ignoreInEval": False, "id": 23, "color": [70, 130, 180], "trainId": 10},
//     {"person", "ignoreInEval": False, "id": 24, "color": [220, 20, 60], "trainId": 11},
//     {"rider", "ignoreInEval": False, "id": 25, "color": [255, 0, 0], "trainId": 12},
//     {"car", "ignoreInEval": False, "id": 26, "color": [0, 0, 142], "trainId": 13},
//     {"truck", "ignoreInEval": False, "id": 27, "color": [0, 0, 70], "trainId": 14},
//     {"bus", "ignoreInEval": False, "id": 28, "color": [0, 60, 100], "trainId": 15},
//     {"caravan", "ignoreInEval": True, "id": 29, "color": [0, 0, 90], "trainId": 255},
//     {"trailer", "ignoreInEval": True, "id": 30, "color": [0, 0, 110], "trainId": 255},
//     {"train", "ignoreInEval": False, "id": 31, "color": [0, 80, 100], "trainId": 16},
//     {"motorcycle", "ignoreInEval": False, "id": 32, "color": [0, 0, 230], "trainId": 17},
//     {"bicycle", "ignoreInEval": False, "id": 33, "color": [119, 11, 32], "trainId": 18},
//     {"license plate", "ignoreInEval": True, "id": -1, "color": [0, 0, 142], "trainId": -1}
// ]

const int class_num = 19;

// static const char* cityscapes_labels[] = {
static std::vector<std::string> cityscapes_labels = {
  "road", "sidewalk", "building", "wall", "fence", 
  "pole", "traffic light", "traffic sign", "vegetation", "terrain",
  "sky", "person", "rider", "car", "truck", 
  "bus", "train", "motorcycle", "bicycle",
};

static vector<int> _classes_colors = {
  128, 64, 128, 
  244, 35, 232, 
  70, 70, 70,
  102, 102, 156,
  190, 153, 153,
  153, 153, 153,
  250, 170, 30,
  220, 220, 0,
  107, 142, 35,
  152, 251, 152,
  70, 130, 180,
  220, 20, 60,
  255, 0, 0,
  0, 0, 142,
  0, 0, 70,
  0, 60, 100,
  0, 80, 100,
  0, 0, 230,
  119, 11, 32
};

bool onnx_hub(const char* name, const char* save_to);

static void render(cv::Mat& image, const UNet::segment_pair& seg){
    
    cv::Mat prob, iclass, invm;
    tie(prob, iclass, invm) = seg;

    cv::warpAffine(prob, prob, invm, image.size(), cv::INTER_LINEAR);
    cv::warpAffine(iclass, iclass, invm, image.size(), cv::INTER_NEAREST);

    auto pimage = image.ptr<cv::Vec3b>(0);
    auto pprob  = prob.ptr<float>(0);
    auto pclass = iclass.ptr<uint8_t>(0);

    for(int i = 0; i < image.cols*image.rows; ++i, ++pimage, ++pprob, ++pclass){

        int iclass        = *pclass;
        float probability = *pprob;
        auto& pixel       = *pimage;
        float foreground  = min(0.6f + probability * 0.2f, 0.8f);
        float background  = 1 - foreground;
        for(int c = 0; c < 3; ++c){
            auto value = pixel[c] * background + foreground * _classes_colors[iclass * 3 + 2-c];
            pixel[c] = std::min<int>(value, 255);
        }
    }
}

template<typename _T>
void worker(_T engine, cv::Mat image){

    typedef decltype(engine->commit(image)) futtype;
    vector<futtype> futs;
    for(int i = 0; i < 100; ++i){
        futs.emplace_back(engine->commit(image));
    }

    for(int i = 0; i < futs.size(); ++i){
        futs[i].get();
    }
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

  CUDAKernel::Norm normalize = CUDAKernel::Norm::alpha_beta(2.0f/255.0f, -1.0f, CUDAKernel::ChannelType::Invert);
  cv::Size input_size(tensor->size(3), tensor->size(2));
  printf("Affine Transform [%dx%d]-->[%dx%d]\n", image.rows, image.cols, tensor->size(2), tensor->size(3));
  AffineMatrix affine;
  affine.compute(image.size(), input_size, true);

  size_t size_image      = image.cols * image.rows * 3;
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
    image_device,               image.cols * 3,       image.cols,       image.rows, 
    tensor->gpu<float>(ibatch), input_size.width,     input_size.height, 
    affine_matrix_device, 128, 
    normalize, stream
  );
  return affine.d2i_mat().clone();
}


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

int main(){

  int deviceid = 0;
  TRT::set_device(deviceid);

  std::unordered_map<int, cv::Vec3b> color_table;

  for(int i = 0; i < class_num; ++i) {
    cv::Vec3b color(_classes_colors[i*3], _classes_colors[i*3+1], _classes_colors[i*3+2]);
    color_table.insert(std::make_pair(i, color));
  }

  for(auto iter = color_table.begin(); iter != color_table.end(); ++iter) {
    std::cout << "key: " << iter->first << " value: " << iter->second << std::endl; 
  }

  if(!iLogger::exists("bisenetv2.trtmodel")){
    printf("bisenetv2.trtmodel doesn't exist! Build the engine!\n");
    TRT::compile(TRT::Mode::FP32, 1, "../bisenetv2.onnx", "bisenetv2.trtmodel");
  }

  std::shared_ptr<TRT::Infer> segmentor = TRT::load_infer("bisenetv2.trtmodel");
  if(segmentor == nullptr) {
    INFOE("Engine is nullptr");
    return -1;
  }

  std::shared_ptr<TRT::Tensor> input = segmentor->tensor("input");
  std::shared_ptr<TRT::Tensor> output = segmentor->tensor("output");  

  int eval_w = input->size(3), eval_h = input->size(2);

  int max_batch_size = 1;
  input->resize_single_dim(0, max_batch_size).to_gpu();  

  int ibatch = 0;
  INFO("input.shape = %s", input->shape_string());
  INFO("output.shape = %s", output->shape_string());
  
  // cv::Mat img = cv::imread("example2.jpg", cv::IMREAD_UNCHANGED);

  std::vector<std::string> image_names = ReadAllImages("/media/summervibe/5/dataset/DrivingStereo/left/2018-07-09-16-11-56/");

  for(int i = 0; i < image_names.size(); ++i) {
    cv::Mat img = cv::imread(image_names[i], cv::IMREAD_UNCHANGED);
    cv::Mat imat = warpAffine(img, input, ibatch);
    double start = cv::getTickCount();
    segmentor->forward(true);
    // printf("Forward Done. %lf s\n", (cv::getTickCount() - start) / cv::getTickFrequency());

    cv::Mat semantic_img = cv::Mat(eval_h, eval_w, CV_32SC1, cv::Scalar(255));
    cv::Mat semantic_vis_img = cv::Mat(eval_h, eval_w, CV_8UC3, cv::Scalar(0, 0, 0));
    int *output_data_ptr = output->cpu<int>(ibatch);
    memcpy((void*)semantic_img.data, (void*)output_data_ptr, eval_h * eval_w * sizeof(int));

    for(int i = 0; i < eval_h; ++i) {
      for(int j = 0; j < eval_w; ++j) {
        semantic_vis_img.at<cv::Vec3b>(i*eval_w + j) = color_table[semantic_img.at<int>(i*eval_w + j)];
      }
    }
    printf("Forward Done. %lf s\n", (cv::getTickCount() - start) / cv::getTickFrequency());
    cv::imshow("semantic img", semantic_vis_img);
    cv::waitKey(1);
  }


  return 0;
}
