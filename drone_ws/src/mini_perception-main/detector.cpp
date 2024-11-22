#include <trt_builder.hpp>
#include <trt_infer.hpp>
#include <ilogger.hpp>
#include <yolo/yolo.hpp>
#include <unet/unet.hpp>
#include <stereo/stereo.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <thread>
#include <dirent.h>

using namespace std;

static const char* cocolabels[] = {
    "person", "bicycle", "car", "motorcycle", "airplane",
    "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
    "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
    "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
};

static const char* bddlabels[] = {
    "person", "rider", "car", "bus", "truck",
    "bike", "motor", "tl_green", "tl_red", "tl_yellow", "tl_none",
    "traffic sign", "train", "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
    "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass",
    "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
};

static vector<int> _classes_colors = {
    0, 0, 0, 
    128, 0, 0, 
    0, 128, 0, 
    128, 128, 0, 
    0, 0, 128, 
    128, 0, 128, 
    0, 128, 128, 
    128, 128, 128, 
    64, 0, 0, 
    192, 0, 0, 
    64, 128, 0, 
    192, 128, 0, 
    64, 0, 128, 
    192, 0, 128, 
    64, 128, 128, 
    192, 128, 128, 
    0, 64, 0,
    128, 64, 0, 
    0, 192, 0, 
    128, 192, 0, 
    0, 64, 128, 
    128, 64, 12
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

    // onnx_hub("yolov5s", "yolov5s.onnx");
    // onnx_hub("unet", "unet.onnx");

    // if(!iLogger::exists("yolov5s.fp32.trtmodel")){
    //     printf("yolov5s.fp32.trtmodel doesn't exist! Build the engine!\n");
    //     TRT::compile(TRT::Mode::FP32, 1, "yolov5s.onnx", "yolov5s.fp32.trtmodel");
    // }

    // if(!iLogger::exists("lanedet.fp32.trtmodel")){
    //     printf("lanedet.fp32.trtmodel doesn't exist! Build the engine!\n");
    //     TRT::compile(TRT::Mode::FP32, 1, "lanedet.onnx", "lanedet.fp32.trtmodel");
    // }

    if(!iLogger::exists("rangenet.fp32.trtmodel")){
        printf("rangenet.fp32.trtmodel doesn't exist! Build the engine!\n");
        TRT::compile(TRT::Mode::FP32, 1, "../darknet53/model.onnx", "rangenet.fp32.trtmodel");
    }

    std::shared_ptr<TRT::Infer> segmentor = TRT::load_infer("rangenet.fp32.trtmodel");
    if(segmentor == nullptr) {
      INFOE("Engine is nullptr");
      return -1;
    }
    return 0;

    // if(!iLogger::exists("unet.trtmodel")){
    //     printf("unet.trtmodel doesn't exist! Build the engine!\n");
    //     TRT::compile(TRT::Mode::FP32, 1, "unet.onnx", "unet.trtmodel");
    // }
    
    bool multistream = false;
    auto det = Yolo::CreateInfer(
      "yolov5s.fp32.trtmodel",                 // engine file
      Yolo::Type::V5,                       // yolo type, Yolo::Type::V5 / Yolo::Type::X
      deviceid,
      0.5, 0.7, Yolo::NMSMethod::FastGPU, 1024, multistream
    );

    // auto seg = UNet::CreateInfer(
    //   "unet.trtmodel", deviceid, multistream
    // );

    // auto stereo = mini_perception::StereoNet::CreateModel("trtstereo_fast.fp32.trtmodel");


    return 0;

        
    printf("Multi thread start.");

    const std::string left_img_path = "/media/summervibe/5/dataset/DrivingStereo/left/2018-07-09-16-11-56";
    const std::string right_img_path = "/media/summervibe/5/dataset/DrivingStereo/right/2018-07-09-16-11-56";

    assert(left_img_path == right_img_path);

    std::vector<std::string> left_names = ReadAllImages(left_img_path);
    std::vector<std::string> right_names = ReadAllImages(right_img_path);

    // vector<thread> ts;
    // for(int i = 0; i < 10; ++i){
    //     // 为每个模型启动10个线程
    //     ts.emplace_back(worker<shared_ptr<Yolo::Infer>>, det, image);
    //     ts.emplace_back(worker<shared_ptr<UNet::Infer>>, seg, image);
    // }

    // for(auto& t : ts)
    //     t.join();

    // int stereo_eval_w = stereo->get_eval_width();
    // int stereo_eval_h = stereo->get_eval_height();
    // cv::Mat disp_vis = cv::Mat(stereo_eval_h, stereo_eval_w, CV_8UC3), disp_8u;

    cv::Mat imgL, imgR;
    for(size_t i = 0; i < left_names.size(); ++i) {
      imgL = cv::imread(left_names[i], cv::IMREAD_UNCHANGED);
      imgR = cv::imread(right_names[i], cv::IMREAD_UNCHANGED);
      // printf("Multi thread done.");
      // 并行的理解，commit函数提交image后立即返回。通过fut.get等待推理结束结果获取
      double start = cv::getTickCount();
      auto boxes_fut = det->commit(imgL);
      // auto stereo_fut = stereo->Inference(imgL, imgR, false);
      // auto seg_fut = seg->commit(imgL);
      auto boxes = boxes_fut.get();
      // cv::Mat disp = stereo_fut.get();
      // auto seg_pair = seg_fut.get();
      printf("Forward Done. %lf s\n", (cv::getTickCount() - start) / cv::getTickFrequency());
      // render(image, seg_pair);
      // disp = disp * 4;
      // disp.convertTo(disp_8u, CV_8U);
      // cv::applyColorMap(disp_8u, disp_vis, cv::COLORMAP_INFERNO);

      for(auto& obj : boxes){
        uint8_t b, g, r;
        tie(b, g, r) = iLogger::random_color(obj.class_label);
        cv::rectangle(imgL, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom), cv::Scalar(b, g, r), 1);

        auto name    = bddlabels[obj.class_label];
        auto caption = iLogger::format("%s %.2f", name, obj.confidence);
        // int width    = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 0;
        // cv::rectangle(image, cv::Point(obj.left, obj.top), cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
        cv::putText(imgL, caption, cv::Point(obj.left, obj.top-5), 0, 0.4, cv::Scalar(b, g, r), 1, 16);
      }
      // cv::imwrite("result.jpg", image);
      cv::imshow("detection", imgL);
      // cv::imshow("stereo", disp_vis);
      // while(1) {
      //   if(cv::waitKey(0) == 'q')
      //     break;
      // }
      cv::waitKey(1);
    }
    return 0;
}