#ifndef UNET_HPP
#define UNET_HPP

#include <vector>
#include <memory>
#include <string>
#include <future>
#include <opencv2/opencv.hpp>
#include <tensorRT/trt_tensor.hpp>
#include <common/object_detector.hpp>

namespace UNet{

    using namespace std;

    typedef tuple<cv::Mat, cv::Mat, cv::Mat> segment_pair;

    void image_to_tensor(const cv::Mat& image, shared_ptr<TRT::Tensor>& tensor, int ibatch);

    class Infer{
    public:
        virtual shared_future<segment_pair> commit(const cv::Mat& image) = 0;
        virtual vector<shared_future<segment_pair>> commits(const vector<cv::Mat>& images) = 0;
    };

    shared_ptr<Infer> CreateInfer(
        const string& engine_file, int gpuid,
        bool use_multi_preprocess_stream = false
    );

}; // namespace UNet

#endif // UNET_HPP