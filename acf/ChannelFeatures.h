/*
 * ChannelFeatures.h
 */
#pragma once

#include <vector>
#include <array>
#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

// Channel-classes
#include "ColorChannel.h"
#include "GradMagChannel.h"
#include "GradHistChannel.h"

/*
 * This class will be used to generate the features. By hiding the implementation details of the channels,
 * we can avoid having memory-leaks due to users who are not familiar with the channel-functions (which are
 *  build around malloc's/calloc's and free's)
 */
class ChannelFeatures {
public:
    friend class SqrtChannelFeatures;

    ChannelFeatures(float* image_yuv, size_t image_width,
            size_t image_height, int shrinking);

    ChannelFeatures(const ChannelFeatures &real_channels, int scaled_width,
            int scaled_height, const std::array<double, 3>& lambdas);

    void SmoothPadAndConcatChannel(int padLR, int padTB);
    virtual ~ChannelFeatures();

    void print_info() const {
        std::cout << channel_width << "x" << channel_height << "x" << n_channels << " /" << shrink;
    }

    void printFeatures(int index) const {

        for (int y = 0; y < channel_height; y++) {
            for (int x = 0; x < channel_width; x++) {
                std::cout << features[index][x * channel_height + y] << " ";
            }
            std::cout << std::endl;
        }
    }

    void showFeature(int index) const {
        cv::Mat feature;
        cv::transpose(
                cv::Mat(this->channel_width, this->channel_height, CV_32FC1,
                        (void *) features[index]), feature);
        cv::imshow("Channel " + std::to_string(index), feature);
        cv::moveWindow("Channel " + std::to_string(index),
                index * this->channel_width, 600);
    }

    void showFeature(int index, int x, int y) const {
        cv::Mat feature;
        cv::transpose(
                cv::Mat(this->channel_width, this->channel_height, CV_32FC1,
                        (void *) features[index]), feature);
        cv::imshow("Channel " + std::to_string(index), feature);
        cv::moveWindow("Channel " + std::to_string(index), x, y);
    }

    void showFeature(int index, std::string name, int x, int y) const {
        cv::Mat feature;
        cv::transpose(
                cv::Mat(this->channel_width, this->channel_height, CV_32FC1,
                        (void *) features[index]), feature);
        cv::imshow(name + std::to_string(index), feature);
        cv::moveWindow(name + std::to_string(index), x, y);
    }

    void saveFeature(int index, std::string filepath) const {
        cv::Mat dst, feature;
        cv::Mat(this->channel_width, this->channel_height, CV_32FC1,
                (void *) features[index]).convertTo(dst, CV_8UC1, 255);
//        cv::transpose(dst, feature);
        cv::imwrite(filepath, dst);
    }

    int getChannelWidth() const {
        return this->channel_width;
    }

    int getChannelHeight() const {
        return this->channel_height;
    }

    int getnChannels() const {
        return this->n_channels;
    }

    float getFeatureValue(int channel, int location) const;

    float *chns = NULL;
    const float *image_luv;
    int color_duration = -1;
    int mag_duration = -1;
    int hist_duration = -1;
    int init_duration = -1;
    int smooth_duration = -1;
    int pad_duration = -1;
private:
    void addChannelFeatures(Channel &ch);
    std::vector<const float*> pointers;
    std::vector<float*> features;
    int shrink;
    int channel_width, channel_height, n_channels;
};

