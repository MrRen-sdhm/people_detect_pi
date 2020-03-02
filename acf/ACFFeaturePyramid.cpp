/*
 * ACFFeaturePyramid.cpp
 */
#include "ACFFeaturePyramid.h"
#include "../low-level/Functions.h"
#include <cmath>
#include <chrono>
#include <tbb/tbb.h>
#include <opencv2/opencv.hpp>
//#include <ctgmath>

#define USE_TBB

ACFFeaturePyramid::ACFFeaturePyramid(const cv::Mat &source_image,
        int _scales_per_oct, cv::Size minSize, float shrink,
        const std::array<double, 3>& lambdas, int pad_width, int pad_height) :
        scales_per_oct(_scales_per_oct), minSize(minSize), image_size(
                source_image.cols, source_image.rows) {

    // 不使用增采样
    int n_oct_upsample = 0;
    // 计算二倍频数量
    float n_oct = n_oct_upsample
            + log2f(
                    std::min((float) image_size.width / minSize.width,
                            (float) image_size.height / minSize.height));
    // 计算尺度数量
    int n_scales = std::floor(
            static_cast<float>(scales_per_oct * (n_oct + n_oct_upsample) + 1));

    // 设定特征金字塔的层数为尺度数量set the size of the feature-pyramid at the nScales amount
    layers.resize(n_scales);

    std::vector<float> origin_scales;
    // 生成各个尺度的缩放比例
    for (int s = 0; s < n_scales; s++) {
        origin_scales.push_back(
                std::pow(2.0, ((float) -s / scales_per_oct + n_oct_upsample)));
    }

    // 优化缩放系数
    float dim_short, dim_long;
    if (image_size.height < image_size.width) {
        dim_short = image_size.height;
        dim_long = image_size.width;
    } else {
        dim_short = image_size.width;
        dim_long = image_size.height;
    }

    float eps = 2.2204e-16;
    for (float& scale : origin_scales) {
        // 长轴与短轴的实际缩放系数
        float scale_short = (round((float) dim_short * scale / shrink) * shrink
                - 0.25 * shrink) / dim_short;
        float scale_long = (round((float) dim_short * scale / shrink) * shrink
                + 0.25 * shrink) / dim_short;

        // 尝试两种缩放系数中的各个值
        std::vector<double> inteploted_scales, scaled_dim_error;
        for (double percent = 0; percent <= 1 - eps; percent += 0.01) {
            // 计算内插的缩放系数
            float interploted_scale = percent * (scale_long - scale_short)
                    + scale_short;
            // 计算该缩放系数对应在长轴与短轴上的偏差
            float scaled_dim_short = interploted_scale * dim_short;
            float scaled_dim_short_error = std::abs(
                    scaled_dim_short
                            - round(scaled_dim_short / shrink) * shrink);

            float scaled_dim_long = interploted_scale * dim_long;
            float scaled_dim_long_error = std::abs(
                    scaled_dim_long - round(scaled_dim_long / shrink) * shrink);

            inteploted_scales.push_back(interploted_scale);
            scaled_dim_error.push_back(
                    std::max(scaled_dim_short_error, scaled_dim_long_error));
        }

        // 找到偏差最小的缩放系数
        int best_index = 0;
        float min_error = scaled_dim_error[0];
        for (int i = 1; i < scaled_dim_error.size(); i++) {
            if (scaled_dim_error[i] < min_error) {
                min_error = scaled_dim_error[i];
                best_index = i;
            }
        }

        // 将缩放系数设定为最佳值
        scale = inteploted_scales[best_index];
        int width = round(image_size.width * scale / shrink) * shrink;
        int height = round(image_size.height * scale / shrink) * shrink;
        cv::Size scaled_size(width, height);
        // 添加到尺寸列表
        if (std::find(scaled_sizes.begin(), scaled_sizes.end(), scaled_size)
                == scaled_sizes.end()) {
            scaled_sizes.push_back(scaled_size);
        }
    }

    // 统计需要计算的真实尺度
    std::vector<bool> is_added_scales(n_scales, false);
    std::vector<std::pair<int, std::vector<int>>> scale_tree;

    int real_scale_size_threshold = 0; // 80000
    for (int i = 0; i < n_scales; i += scales_per_oct) {
        // 统计属下的估计尺度
        std::vector<int> sub_scales;
        int front = i - (scales_per_oct - 1) / 2;
        int back = i + scales_per_oct / 2;
        if (front < 0) {
            front = 0;
        }
        if (back >= n_scales) {
            back = n_scales - 1;
        }
        for (int j = front; j <= back; j++) {
            if (j != i
                    && scaled_sizes[j].width * scaled_sizes[j].height
                            >= real_scale_size_threshold) {
                sub_scales.push_back(j);
                is_added_scales[j] = true;
            }
        }
        // 优先添加具有下属估计尺度的真实尺度
        if (!sub_scales.empty()) {
            scale_tree.emplace_back(i, sub_scales);
            is_added_scales[i] = true;
        }
    }
    // 添加剩余的尺度
    for (int i = 0; i < n_scales; i++) {
        if (!is_added_scales[i]) {
            scale_tree.emplace_back(i, std::vector<int>());
        }
    }

    // 打印尺度的关系
//    for (const auto& p : scale_tree) {
//        print_scale(p.first);
//        std::cout << ":";
//        for (const auto& i : p.second) {
//            std::cout << " ";
//            print_scale(i);
//        }
//        std::cout << std::endl;
//    }

    // 使用OpenCV的转置函数, 将数据排列转换为按列存储
    cv::Mat mat_temp = cv::Mat(image_size.width, image_size.height, CV_8UC3);
    cv::transpose(source_image, mat_temp);
    // 将图像转换到YUV颜色空间, 存储格式为float数组,
    float *image_luv = (float *) aligned_alloc(16,
            image_size.width * image_size.height * 3 * sizeof(float));
    assert(image_luv != NULL);

    /* 使用cvtColor进行转换, >23ms, 更耗时 */
    auto measure_time = std::chrono::high_resolution_clock::now();
//    cv::Mat mat_luv(image_size.width, image_size.height, CV_32FC3, image_luv);
//    cv::cvtColor(mat_temp / 255.0f, mat_luv, CV_BGR2Luv);
//    std::cout << "cvtColor cost "
//            << std::chrono::duration<float>(
//                    std::chrono::high_resolution_clock::now() - measure_time).count()
//                    * 1000 << std::endl;
    /* 使用rgb2luv_sse进行转换, 17ms, 更快速  */
    // 将图像转换为Matlab形式存储: float数组, 分为R G B通道, 每个通道width列, 每列height像素
    // 预申请对齐的内存空间
    uint8_t *image_matlab_format_data = (uint8_t *) aligned_alloc(16,
            image_size.width * image_size.height * 3 * sizeof(uint8_t));
    assert(image_matlab_format_data != NULL);
    // 使用OpenCV的split函数, 将BGR像素格式拆分为R通道, G通道, B通道
    cv::Mat image_channel_bgr[3] = { cv::Mat(image_size.width,
            image_size.height, CV_8UC1,
            image_matlab_format_data
                    + image_size.width * image_size.height * 2), cv::Mat(
            image_size.width, image_size.height, CV_8UC1,
            image_matlab_format_data + image_size.width * image_size.height),
            cv::Mat(image_size.width, image_size.height, CV_8UC1,
                    image_matlab_format_data), };
    cv::split(mat_temp, image_channel_bgr);
//    auto measure_time = std::chrono::high_resolution_clock::now();
    rgb2luv_sse(image_matlab_format_data, image_luv,
            image_size.height * image_size.width, 1.0f / 255);
//    std::cout << "rgb2luv_sse cost "
//            << std::chrono::duration<float>(
//                    std::chrono::high_resolution_clock::now() - measure_time).count()
//                    * 1000 << std::endl;
    free(image_matlab_format_data);

    pre_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    measure_time = std::chrono::high_resolution_clock::now();
    // 计算实际尺度的特征图 Real Scales
#ifdef USE_TBB
    tbb::parallel_for(size_t(0), scale_tree.size(),
            [&scale_tree, image_luv, shrink, lambdas, pad_width, pad_height, this](size_t i) {
#else
            for (int i = 0; i < scale_tree.size(); i++) {
#endif
            auto measure_time = std::chrono::high_resolution_clock::now();

            int real_scale_i = scale_tree[i].first;
            std::vector<int>& sub_scales = scale_tree[i].second;

            // 计算缩放后的图像尺寸
            cv::Size &real_scale_size = scaled_sizes[real_scale_i];
            int scaled_width = real_scale_size.width;
            int scaled_height = real_scale_size.height;

            // 申请对齐内存
            float *scaled_image = (float *) aligned_alloc(16,
                    scaled_width * scaled_height * 3 * sizeof(float));
            if (scaled_image == NULL) {
                throw std::runtime_error("Failed to aligned_alloc scaled_image");
            }

            // 计算缩放后的图像
            // resize(opencv) is >4x faster than resample(pdollar toolbox)
            // 使用opencv resize
            for (size_t n = 0; n < 3; n++) {
                cv::Mat src_mat = cv::Mat(image_size.width, image_size.height, CV_32FC1,
                        image_luv + n * image_size.width * image_size.height);
                cv::Mat scaled_mat = cv::Mat(scaled_width, scaled_height, CV_32FC1,
                        scaled_image + n * scaled_height * scaled_width);
                cv::resize(src_mat, scaled_mat,
                        cv::Size(scaled_height, scaled_width));
            }
            int resize_dur =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now()
                    - measure_time).count();

            // 计算实际尺度下的特征图
            layers[real_scale_i] = new ChannelFeatures(scaled_image, scaled_width,
                    scaled_height, shrink);
            layers[real_scale_i]->init_duration += resize_dur;
//            if (scaled_height == 240)
//                std::cout << "ChannelFeatures cost "
//                        << std::chrono::duration<float>(
//                                std::chrono::high_resolution_clock::now()
//                                        - measure_time).count() * 1000 << std::endl;
            // 计算该实际尺度所对应的估计尺度的特征图
#ifdef USE_TBB
            tbb::parallel_for(size_t(0), sub_scales.size(), [&sub_scales, real_scale_i, lambdas, pad_width, pad_height, shrink, this](size_t i) {
#else
            for (int i = 0; i < sub_scales.size(); i++) {
#endif
            int sub_scale_i = sub_scales[i];
            layers[sub_scale_i] = new ChannelFeatures(*layers[real_scale_i], scaled_sizes[sub_scale_i].width, scaled_sizes[sub_scale_i].height,
                    lambdas);
            // 特征图后处理
            layers[sub_scale_i]->SmoothPadAndConcatChannel(pad_width / shrink,
                    pad_height / shrink);
#ifdef USE_TBB
        });
#else
        }
#endif
            // 特征图后处理
            layers[real_scale_i]->SmoothPadAndConcatChannel(pad_width / shrink,
                    pad_height / shrink);
#ifdef USE_TBB
        });
#else
        }
#endif
            free(image_luv)
;    //    std::cout << "Layers cost "
//            << std::chrono::duration<float>(
//                    std::chrono::high_resolution_clock::now() - measure_time).count()
//                    * 1000 << std::endl;

    calc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    // 保存特征图
//    std::system("mkdir -p /home/pi/features/");
//    std::system("rm /home/pi/features/*");
//    for (size_t i = 0; i < layers.size(); i ++) {
//        for (int n = 0; n < 10; n++) {
//            layers[i]->saveFeature(n, "/home/pi/features/L" + std::to_string(i) + "F" + std::to_string(n) + ".png");
//        }
//        std::cout << "[" << i << "] ";
//        layers[i]->print_info();
//        std::cout << std::endl;
//    }
//    std::cout << "Features saved" << std::endl;

}

ACFFeaturePyramid::~ACFFeaturePyramid() {
    for (auto& layer : layers) {
        if (layer != NULL) {
            if (layer->image_luv != NULL) {
                free((void *) layer->image_luv);
                layer->image_luv = NULL;
            }
            delete layer;
            layer = NULL;
        }
    }
}

