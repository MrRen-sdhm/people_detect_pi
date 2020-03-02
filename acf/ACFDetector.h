/*
 * ACFDetector.h
 */

#ifndef ACFDETECTOR_H_
#define ACFDETECTOR_H_

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "../general/detection.h"
#include "../general/DetectionList.h"

#include "ChannelFeatures.h"

#include "ACFFeaturePyramid.h"

class ACFDetector {
public:

    std::string getName() const {
        return "ACF";
    }

    ACFDetector() {
        ReadModel("/home/pi/AcfHSCaltechDetector.mat");
    }
    ~ACFDetector();

    std::vector<Detection> Detect(const ChannelFeatures *features) const;

    int getShrinking() const {
        return this->shrinking;
    }

    void modifyCascade(float score) {
//        for(int s=0; s<this->Values.size(); s++) {
//            for(int l=0; l<this->Values[s].size(); l++) {
//                this->Values[s][l] += score;
//            }
//        }
    }

    ACFDetector(std::string modelfile);

    DetectionList applyDetector(const cv::Mat &Frame);

    int getHeight() const {
        return this->model_height;
    }
    int pad_width;
    int pad_height;
    std::array<double, 3> lambdas;


    ACFFeaturePyramid *feature_pyramid = NULL;
    int calc_feature_ms = 0;
    int apply_classifier_ms = 0;

private:

    void setWidth(float w) {
        this->model_width = w;

    }

    void setHeight(float h) {
        this->model_height = h;
    }

    void setWidthPad(float w) {
        this->model_width_pad = w;

    }

    void setHeightPad(float h) {
        this->model_height_pad = h;
    }

    void ReadModel(std::string modelfile);

    int nTrees;

    int nTreeNodes;

    //! holds indeces to follow through a stage evaluation, normally these are the same in every stage since these represent the decision stump tree
    const uint32_t **Child;

    //! holds indeces to follow through a stage evaluation, normally these are the same in every stage since these represent the decision stump tree
    // std::vector<std::vector<int> > depth;

    //! holds the indeces of the features to use, no longer used when features are inside the model
    const uint32_t **Fid;

    //! The thresholds that should be reached for each feature
    const float **Thresholds;

    //! stores the values to be added/subtracted at the end of a stage (dependent on the leaf reached)
    const float **Values;

    float model_width, model_height;
    float model_width_pad, model_height_pad;
    int shrinking;
    double cascThr;
    int ModelDepth;

    float *thrs = NULL;
    float *hs = NULL;
    uint32_t *fids = NULL;
    uint32_t *child = NULL;
};

#endif /* ACFDETECTOR_H_ */
