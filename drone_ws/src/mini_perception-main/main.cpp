#include <builder/trt_builder.hpp>
#include <infer/trt_infer.hpp>
#include <common/ilogger.hpp>
#include <yolo/yolo.hpp>
#include <unet/unet.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <thread>

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

static vector<int> _classes_colors = {
    0, 0, 0, 128, 0, 0, 0, 128, 0, 128, 128, 0, 0, 0, 128, 128, 0, 128, 0, 128, 128, 
    128, 128, 128, 64, 0, 0, 192, 0, 0, 64, 128, 0, 192, 128, 0, 64, 0, 128, 192, 0, 128, 
    64, 128, 128, 192, 128, 128, 0, 64, 0, 128, 64, 0, 0, 192, 0, 128, 192, 0, 0, 64, 128, 128, 64, 12
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

int main(){

    int deviceid = 0;
    TRT::set_device(deviceid);

    // onnx_hub("yolov5s", "yolov5s.onnx");
    // onnx_hub("unet", "unet.onnx");

    if(!iLogger::exists("yolov5s.trtmodel")){
        printf("yolov5s.trtmodel doesn't exist! Build the engine!\n");
        TRT::compile(TRT::Mode::FP32, 1, "yolov5s.onnx", "yolov5s.trtmodel");
    }

    if(!iLogger::exists("unet.trtmodel")){
        printf("unet.trtmodel doesn't exist! Build the engine!\n");
        TRT::compile(TRT::Mode::FP32, 1, "unet.onnx", "unet.trtmodel");
    }
    
    bool multistream = false;
    auto det1 = Yolo::CreateInfer(
        "yolov5s.trtmodel",                 // engine file
        Yolo::Type::V5,                       // yolo type, Yolo::Type::V5 / Yolo::Type::X
        deviceid,
        0.25, 0.5, Yolo::NMSMethod::FastGPU, 1024, multistream
    );

    auto det2 = Yolo::CreateInfer(
        "yolov5s.trtmodel",                 // engine file
        Yolo::Type::V5,                       // yolo type, Yolo::Type::V5 / Yolo::Type::X
        deviceid,
        0.25, 0.5, Yolo::NMSMethod::FastGPU, 1024, multistream
    );

    auto seg = UNet::CreateInfer(
        "unet.trtmodel", deviceid, multistream
    );

    auto image = cv::imread("street.jpg");
    if(image.empty()) {
        printf("Empty Image\n");
        return -1;
    }
        
    printf("Multi thread start.");

    vector<thread> ts;
    for(int i = 0; i < 10; ++i){
        // 为每个模型启动10个线程
        ts.emplace_back(worker<shared_ptr<Yolo::Infer>>, det1, image);
        ts.emplace_back(worker<shared_ptr<Yolo::Infer>>, det2, image);
        ts.emplace_back(worker<shared_ptr<UNet::Infer>>, seg, image);
    }

    for(auto& t : ts)
        t.join();

    printf("Multi thread done.");
    // 并行的理解，commit函数提交image后立即返回。通过fut.get等待推理结束结果获取
    auto boxes1_fut = det1->commit(image);
    auto boxes2_fut = det2->commit(image);
    auto seg_fut = seg->commit(image);
    auto boxes1 = boxes1_fut.get();
    auto boxes2 = boxes2_fut.get();
    auto seg_pair = seg_fut.get();
    render(image, seg_pair);

    for(auto& obj : boxes1){
        uint8_t b, g, r;
        tie(b, g, r) = iLogger::random_color(obj.class_label);
        cv::rectangle(image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom), cv::Scalar(b, g, r), 5);

        auto name    = cocolabels[obj.class_label];
        auto caption = iLogger::format("%s %.2f", name, obj.confidence);
        int width    = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
        cv::rectangle(image, cv::Point(obj.left-3, obj.top-33), cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
        cv::putText(image, caption, cv::Point(obj.left, obj.top-5), 0, 1, cv::Scalar::all(0), 2, 16);
    }
    cv::imwrite("result.jpg", image);
    return 0;
}