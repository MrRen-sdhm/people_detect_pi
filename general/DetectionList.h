#pragma once

#include <vector>
#include "detection.h"
#include <string>
#include <iostream>
#include <iomanip>

// Class to protect from memory-leaks when not releasing detections
class DetectionList {
    // Some classes that should have direct access to the detections
    friend class NonMaximumSuppression;
    friend class Combinator;

public:
    DetectionList();

    ~DetectionList();

    void addDetection(const Detection &det) {
        detections.push_back(Detection(det));
    }

// Move detections
    void moveDetections(float x, float y) {
        for (int d = 0; d < detections.size(); d++) {
            detections[d].setX(detections[d].getX() + x);
            detections[d].setY(detections[d].getY() + y);
        }
    }

    // obtain the number of detections in the list
    int getSize() const {
        return this->detections.size();
    }

    float maxScore() {
        float max = -1;
        for (int d = 0; d < detections.size(); d++) {
            float score = detections[d].getScore();
            if (detections[d].getScore() > max) {
                max = detections[d].getScore();
            }
        }
        return max;
    }

    // Draw the detections on Frame
    int Draw(cv::Mat &Frame, float score_threashold, int line_width = 2) {
        int cound_good = 0;
        for (int d = 0; d < detections.size(); d++) {
            int x = detections[d].getX();
            int y = detections[d].getY();
            int w = detections[d].getWidth();
            int h = detections[d].getHeight();
            float score = detections[d].getScore();
            int level = detections[d].getLevel();
            cv::Scalar color;
            if (score >= score_threashold / 2) {
                color = cv::Scalar(0, 255, 0);
                cound_good++;
            } else {
                color = cv::Scalar(0, 255, 255);
            }
            if (x + w > Frame.cols) {
                w = Frame.cols - x;
            }
            if (y + h > Frame.rows) {
                h = Frame.rows - y;
            }
            cv::Mat roi = Frame(cv::Rect(x, y, w, h));
            cv::Mat temp;
            roi.copyTo(temp);
            cv::rectangle(temp, cv::Point(0, 0),
                    cv::Point(w - line_width, h - line_width),
                    color, line_width);
            float alpha = score >= score_threashold || score < 0 ? 1 : score / score_threashold;
            cv::addWeighted(temp, alpha, roi, 1 - alpha , 0, roi);
            std::stringstream info;
            info << std::fixed << std::setprecision(1) << score << " " << level << " " << Frame.cols / w;
            cv::putText(Frame,
                    info.str(),
                    cv::Point(x, y), 1, 1,
                    cv::Scalar(0, 0, 255));
        }
        return cound_good;
    }

    DetectionList filterSize(float min_width, float min_height) {
        DetectionList new_det;
        int n = this->detections.size();
        for (int i = 0; i < n; i++) {
            if (detections[i].getWidth() >= min_width && detections[i].getHeight() >= min_height) {
                new_det.addDetection(detections[i]);
            }
        }
        return new_det;
    }

    void resizeDetections(float x_scale, float y_scale) {
        for (int d = 0; d < this->detections.size(); d++) {
            detections[d].setX(detections[d].getX() * x_scale);
            detections[d].setY(detections[d].getY() * y_scale);
            detections[d].setWidth(detections[d].getWidth() * x_scale);
            detections[d].setHeight(detections[d].getHeight() * y_scale);
        }
    }

//    DetectionList& operator=(const DetectionList& DL) {
//        this->detections.clear();
//        for (int d = 0; d < DL.getSize(); d++) {
//            Detection *det = new Detection(DL.detections[d]);
//            this->detections.push_back(det);
//        }
//        return *this;
//    }

    static void print_list(const std::vector<Detection>& detections) {
        std::cout << "Detections:" << std::endl;
        for (int d = 0; d < detections.size(); d++) {
            std::cout << "[" << d << "]: ";
            detections[d].print();
        }
    }

    // The detections
    std::vector<Detection> detections;

private:
    // the path to the image
    std::string path;

};

