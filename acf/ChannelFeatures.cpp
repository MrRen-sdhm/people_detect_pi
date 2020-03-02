/*
 * ChannelFeatures.cpp
 */

#include <cstring>
#include <chrono>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <tbb/tbb.h>

#include "../low-level/Functions.h"
#include "ChannelFeatures.h"

#define USE_TBB

// 直接计算多通道特征图, 输入图像的释放不由ChannelFeatures处理
ChannelFeatures::ChannelFeatures(float* image_yuv, size_t image_width,
        size_t image_height, int _shrink) :
        image_luv(image_yuv), shrink(_shrink), channel_height(
                image_height / _shrink), channel_width(image_width / _shrink), n_channels(
                0), chns(NULL) {
    auto measure_time = std::chrono::high_resolution_clock::now();
    ColorChannel luv_channel(image_yuv, image_width, image_height);
    color_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    measure_time = std::chrono::high_resolution_clock::now();
    GradMagChannel grad_mag_channel(luv_channel);
    mag_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    measure_time = std::chrono::high_resolution_clock::now();
    GradHistChannel grad_hist_channel(grad_mag_channel, this->shrink);
    hist_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    measure_time = std::chrono::high_resolution_clock::now();
    this->addChannelFeatures(luv_channel);
    color_duration += std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    measure_time = std::chrono::high_resolution_clock::now();
    this->addChannelFeatures(grad_mag_channel);
    mag_duration += std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    measure_time = std::chrono::high_resolution_clock::now();
    this->addChannelFeatures(grad_hist_channel);
    hist_duration += std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time).count();

    init_duration = color_duration + mag_duration + hist_duration;
}

// 添加通道特征, 若尺寸不匹配则进行降采样
void ChannelFeatures::addChannelFeatures(Channel &ch) {
    float *data;
    if (ch.getHeight() != channel_height || ch.getWidth() != channel_width) {
        auto measure_time = std::chrono::high_resolution_clock::now();
        // 需要降采样
        data = (float*) aligned_alloc(16,
                this->channel_width * this->channel_height * ch.getnChns()
                        * sizeof(float));
        if (data == NULL) {
            throw std::runtime_error("Failed to aligned_alloc data");
        }

#if true
        cv::Mat source_mat;
        cv::Mat scaled_mat;
        for (size_t i = 0; i < ch.getnChns(); i++) {
            source_mat = cv::Mat(ch.getWidth(), ch.getHeight(), CV_32FC1,
                    (float *) ch.getData()
                            + ch.getWidth() * ch.getHeight() * i);
            scaled_mat = cv::Mat(this->channel_width, this->channel_height,
            CV_32FC1,
                    (float *) data
                            + this->channel_width * this->channel_height * i);
            cv::resize(source_mat, scaled_mat,
                    cv::Size(this->channel_height, this->channel_width));
        }
#else
        resample<float>((float *) ch.getData(), (float *) data,
                ch.getHeight(), this->channel_height,
                ch.getWidth(), this->channel_width,
                ch.getnChns(), 1);
#endif

        if (ch.getData() != this->image_luv) {
            free((void *) ch.getData());
        }

        int dur = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - measure_time).count();
//        std::cout << "addChannelFeatures() need down-sample from "
//                << ch.getWidth() << "x" << ch.getHeight() << " to "
//                << channel_width << "x" << channel_height << " " << dur << "ms" << std::endl;
    } else {
        data = ch.getData();
    }
    for (int c = 0; c < ch.getnChns(); c++) {
        this->features.push_back(
                data + c * this->channel_height * this->channel_width);
    }
    if (data != this->image_luv) {
        this->pointers.push_back(data);
    }
    this->n_channels += ch.getnChns();
}

// 通过降采样计算多通道特征图, lambdas: {lambdas_LUV, lambdas_GradMag, lambdas_GradHist}
ChannelFeatures::ChannelFeatures(const ChannelFeatures &real_channels,
        int scaled_width, int scaled_height,
        const std::array<double, 3>& lambdas) :
        image_luv(NULL), channel_width(scaled_width / real_channels.shrink), channel_height(
                scaled_height / real_channels.shrink), shrink(
                real_channels.shrink), n_channels(real_channels.n_channels) {
    auto measure_time = std::chrono::high_resolution_clock::now();

    // 计算各个通道的系数
    float scale_NofR = (((float) channel_width / real_channels.channel_width)
            + ((float) channel_height / real_channels.channel_height)) / 2;
    float ratioLUV = pow(scale_NofR, lambdas[0]);
    float ratioGradMag = pow(scale_NofR, lambdas[1]);
    float ratioGradHist = pow(scale_NofR, lambdas[2]);
    std::array<double, 10> ratios = { ratioLUV, ratioLUV, ratioLUV,
            ratioGradMag, ratioGradHist, ratioGradHist, ratioGradHist,
            ratioGradHist, ratioGradHist, ratioGradHist, };
    if (this->n_channels != 10) {
        throw std::runtime_error("n_channels != 10");
    }

    for (size_t i = 0; i < this->n_channels; i++) {
        // 申请连续的内存空间
        float* channel = (float*) aligned_alloc(16,
                channel_width * channel_height * sizeof(float));
        if (channel == NULL) {
            throw std::runtime_error("Failed to aligned_alloc channel");
        }
        features.push_back(channel);
        pointers.push_back(channel);
    }

    // 以给定的系数进行特征图的重采样
#ifdef USE_TBB
    tbb::parallel_for(size_t(0), size_t(n_channels),
            [this, &ratios, &real_channels](size_t i) {
#else
            for (size_t i = 0; i < n_channels; i++) {
#endif
            const cv::Mat source_mat = cv::Mat(real_channels.channel_width,
                    real_channels.channel_height, CV_32FC1,
                    (float*)real_channels.features[i]);
            cv::Mat scaled_mat = cv::Mat(channel_width, channel_height, CV_32FC1,
                    features[i]);
            cv::resize(source_mat, scaled_mat,
                    cv::Size(channel_height, channel_width));
            if (std::abs(ratios[i] - 1.0) > 0.001) {
                scaled_mat *= ratios[i];
            }
#ifdef USE_TBB
        });
#else
        }
#endif

            init_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - measure_time)
.    count();
}

// 连接所有通道, 并填充边缘像素
void ChannelFeatures::SmoothPadAndConcatChannel(int padLR, int padTB) {

    // 申请连续的对齐内存空间, 便于检测器读取
    this->chns = (float*) aligned_alloc(16,
            (this->channel_width + padLR * 2)
                    * (this->channel_height + padTB * 2) * this->getnChannels()
                    * sizeof(float));
    if (this->chns == NULL) {
        throw std::runtime_error("Failed to aligned_alloc chns");
    }

    // serial 157ms, par 133ms
    smooth_duration = 0;
    pad_duration = 0;
#ifdef USE_TBB
    tbb::parallel_for(size_t(0), size_t(n_channels), [&](size_t i) {
#else
            for (int i = 0; i < n_channels; i++) {
#endif
            auto measure_time = std::chrono::high_resolution_clock::now();

            float* smoothed = (float*) aligned_alloc(16,
                    channel_width * channel_height * sizeof(float));
            if (smoothed == NULL) {
                throw std::runtime_error("Failed to aligned_alloc smoothed");
            }

            // 平滑图像
            convTri1((float *) this->features[i], smoothed, channel_height,
                    channel_width, 1, 2, 1);
//            memcpy(smoothed, this->features[i], channel_width * channel_height * sizeof(float));

            smooth_duration += std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - measure_time).count();

            measure_time = std::chrono::high_resolution_clock::now();

// 调用OpenCV函数进行图像填充, 准备cv::Mat对象
            cv::Mat src = cv::Mat(this->channel_width, this->channel_height,
                    CV_32FC1, smoothed);
            float *dst_data = this->chns
            + (this->channel_width + padLR * 2)
            * (this->channel_height + padTB * 2) * i;
            cv::Mat dst = cv::Mat(this->channel_width + padLR * 2,
                    this->channel_height + padTB * 2,
                    CV_32FC1, dst_data);

            if (i < 3) {
                // 前三个通道位颜色通道, 采用复制方式进行填充
                cv::copyMakeBorder(src, dst, padLR, padLR, padTB, padTB,
                        cv::BORDER_REPLICATE);
            } else {
                // 其余通道为梯度通道, 采用0填充
                cv::copyMakeBorder(src, dst, padLR, padLR, padTB, padTB,
                        cv::BORDER_CONSTANT, 0);
            }

            pad_duration += std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - measure_time).count();

            free(smoothed);
            if (!dst.isContinuous()) {
                throw std::runtime_error("cv::Mat dst is not continuous");
            }
            if ((float * )dst.data != dst_data) {
                throw std::runtime_error("dst.data != dst_data");
            }
            features[i] = dst_data;
#ifdef USE_TBB
        });
#else
        }
#endif

            // 修改通道尺寸
            this->channel_height += 2 * padTB;
            this->channel_width += 2 * padLR;

            // 释放之前数据指针
            for (size_t i = 0; i < this->pointers.size(); i++)
{    free((void*) this->pointers[i]);
}
    this->pointers.clear();

    // 记录数据指针
    this->pointers.push_back(this->chns);
}

ChannelFeatures::~ChannelFeatures() {
    // 释放数据指针
    for (size_t i = 0; i < this->pointers.size(); i++) {
        if (this->pointers[i] != NULL) {
            free((void*) this->pointers[i]);
        }
    }
}

