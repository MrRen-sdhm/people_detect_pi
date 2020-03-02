/*
 * ColorChannel.cpp
 */

#include "ColorChannel.h"
#include "GradMagChannel.h"
#include "GradHistChannel.h"
#include "ChannelFeatures.h"

#include "../low-level/Functions.h"
#include <chrono>
#include <sstream>

// data指针的释放由外部负责
ColorChannel::ColorChannel(float* image_yuv, size_t image_width,
        size_t image_height) {
    this->setChanneldata(image_yuv, image_width, image_height, 3);
}

ColorChannel::~ColorChannel() {

}

// data指针的释放由外部负责
GradMagChannel::GradMagChannel(const ColorChannel &color_channel) {

    std::stringstream oss;

    this->height = color_channel.getHeight();
    this->width = color_channel.getWidth();
    this->nChns = 1;

//    auto measure_time = std::chrono::high_resolution_clock::now();
    data = (float*) aligned_alloc(16,
            width * height * this->nChns * sizeof(float)); // only one channel deep
    if (data == NULL) {
        throw std::runtime_error("Failed to aligned_alloc data");
    }
    orientation = (float*) aligned_alloc(16,
            width * height * this->nChns * sizeof(float));
    if (orientation == NULL) {
        throw std::runtime_error("Failed to aligned_alloc orientation");
    }

    // 计算梯度幅值和梯度方向, 仅计算Y通道上的幅值梯度, 梯度方向的角度范围为[0,pi)
    gradMag((float *) color_channel.getData(), (float *) data,
            (float *) orientation, height, width, 1, 0);
//    if (height == 120)
//        std::cout << "gradMag cost "
//                << std::chrono::duration<float>(
//                        std::chrono::high_resolution_clock::now()
//                                - measure_time).count() * 1000 << std::endl;

//    measure_time = std::chrono::high_resolution_clock::now();
    float *S = (float*) aligned_alloc(16,
            width * height * 1 * sizeof(float));
    if (S == NULL) {
        throw std::runtime_error("Failed to aligned_alloc S");
    }

    // 计算归一化系数图, 半径为5
    convTri((float *) this->data, S, this->height, this->width, 1, 5, 1);
    // 进行归一化, 归一化系数为0.005
    gradMagNorm((float *) this->data, S, this->height, this->width, 0.0050);
//    if (height == 120)
//        std::cout << "convTri gradMagNorm cost "
//                << std::chrono::duration<float>(
//                        std::chrono::high_resolution_clock::now()
//                                - measure_time).count() * 1000 << std::endl;
    free(S);
}

GradMagChannel::~GradMagChannel() {
    free((void *) this->orientation);
}

// data指针的释放由外部负责
GradHistChannel::GradHistChannel(const GradMagChannel &grad_mag_channel,
        uint32_t shrink) {

    this->height = grad_mag_channel.getHeight() / shrink;
    this->width = grad_mag_channel.getWidth() / shrink;
    this->nChns = 6;

    //It is important that this memory is set to zero to avoid random values in the result
    this->data = (float*) aligned_alloc(16,
            this->width * this->height * this->nChns * sizeof(float));
    memset((void *) this->data, 0,
            this->width * this->height * this->nChns * sizeof(float));

    //Parameters, should be configurable
    int block_size = shrink; //equals shrinking
    bool full_2pi = false;

//    auto measure_time = std::chrono::high_resolution_clock::now();
    // 计算梯度方向特征图
    gradHist((float *) grad_mag_channel.getMagnitude(),
            (float *) grad_mag_channel.getOrientation(), (float *) this->data,
            grad_mag_channel.getHeight(), grad_mag_channel.getWidth(), block_size,
            this->nChns, full_2pi);
//    if (height == 120)
//        std::cout << "gradHist cost "
//                << std::chrono::duration<float>(
//                        std::chrono::high_resolution_clock::now()
//                                - measure_time).count() * 1000 << std::endl;

}

GradHistChannel::~GradHistChannel() {
}
