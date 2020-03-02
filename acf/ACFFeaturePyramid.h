/*
 * ACFFeaturePyramid.h
 */
#pragma once

#include "ChannelFeatures.h"

class ACFFeaturePyramid {
public:

    ACFFeaturePyramid(const cv::Mat &source_image, int _scales_per_oct,
            cv::Size minSize, float shrink,
            const std::array<double, 3>& lambdas, int pad_width,
            int pad_height);

    void update(const cv::Mat &source_image);

    virtual ~ACFFeaturePyramid();

    int getAmount() {
        return this->layers.size();
    }

    ChannelFeatures* getLayer(int L) {
        if (L < this->layers.size()) {
            return this->layers[L];
        } else {
            std::cerr << "Requesting unknown layer ..." << std::endl;
            exit(1);
        }
    }

    cv::Size2d get_scale_xy(int i) {
        cv::Size2d scale;
        scale.width = this->scaled_sizes.at(i).width
                / (double) image_size.width;
        scale.height = this->scaled_sizes.at(i).height
                / (double) image_size.height;
        return scale;
    }

    double get_scale(int i) {
        cv::Size2d scale_xy = get_scale_xy(i);
        return (scale_xy.width + scale_xy.height) / 2;
    }

    void print_scale(int i) {
        std::cout << "[" << i << " " << this->scaled_sizes.at(i).width << "x"
                << this->scaled_sizes.at(i).height << "]";
    }

    void print_duration() {
        std::cout << "Pre: " << pre_duration << std::endl;
        int sum_layer_init = 0;
        int sum_layer_smooth = 0;
        int sum_layer_pad = 0;
        for (int i = 0; i < layers.size(); i++) {
            std::cout << "Layer " << i << ": ";
            if (layers[i] == NULL) {
                std::cout << "NULL";
            } else {
                sum_layer_init += layers[i]->init_duration;
                sum_layer_smooth += layers[i]->smooth_duration;
                sum_layer_pad += layers[i]->pad_duration;
                int sum = layers[i]->init_duration + layers[i]->smooth_duration + layers[i]->pad_duration;
                std::cout << "(" << layers[i]->color_duration << " + "
                        << layers[i]->mag_duration << " + "
                        << layers[i]->hist_duration << " = "
                        << layers[i]->init_duration << ") + "
                        << layers[i]->smooth_duration << " + "
                        << layers[i]->pad_duration << " = " << sum;
            }
            std::cout << std::endl;
        }
        std::cout << "Calc: " << calc_duration << " / (" << sum_layer_init
                << " + " << sum_layer_smooth << " + " << sum_layer_pad << ")"
                << std::endl;
    }

    int pre_duration;
    int calc_duration;

protected:

    std::vector<ChannelFeatures*> layers;

    // amount of scales in each octave (so between halving each image dimension )
    int scales_per_oct;
    // Minimum size an image can have (size of the model)
    cv::Size minSize;

    cv::Size image_size;
    std::vector<cv::Size> scaled_sizes;
};
