#pragma once

#include <iostream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

/*!
 \file Declaration of the Detection class
 */

/*!
 This class represents a detection. It can also be used for annotations. This class forms the input and output for our detection/training-software. By making this format detector-independend we simplify the switching between different detectors
 */

class Detection {
public:
    Detection();
    Detection(float x, float y, float width, float height, float score) {
        this->setX(x);
        this->setY(y);
        this->setWidth(width);
        this->setHeight(height);
        this->setScore(score);
        this->setLevel(0);
    }

    Detection(float x, float y, float width, float height, float score,
            std::string name) {
        this->setX(x);
        this->setY(y);
        this->setWidth(width);
        this->setHeight(height);
        this->setScore(score);
        this->setLevel(0);
        this->setModelName(name);
        this->setDetectorName(name);
    }

    Detection(Detection *D) {
        this->setX(D->getX());
        this->setY(D->getY());
        this->setWidth(D->getWidth());
        this->setHeight(D->getHeight());
        this->setScore(D->getScore());
        this->setColor(D->getColor());
        this->setLevel(D->getLevel());

        this->setDetectorName(D->getDetectorName());
    }

    float getX() const;
    float getY() const;
    float getWidth() const;
    float getHeight() const;
    float getScore() const;
    std::string getFilename() const;
    cv::Scalar getColor() const;

    std::string getModelName() const;
    int getLevel() const;

    void setX(float x);

    void setY(float y);

    void setLevel(int level);

    void setWidth(float width);

    void setHeight(float height);

    void setScore(float score);

    void setFilename(std::string filename);

    void setColor(cv::Scalar color);

    void setModelName(std::string modelname);

    void resize(float factor) {
        this->setX(this->getX() / factor);
        this->setY(this->getY() / factor);
        this->setWidth(this->getWidth() / factor);
        this->setHeight(this->getHeight() / factor);
    }

    Detection(const Detection& D);

    void setDetectorName(std::string d) {
        this->detectorname = d;
    }

    std::string getDetectorName() const {
        return this->detectorname;
    }

    cv::Point getCenterPoint() {
        return cv::Point(this->getX() + this->getWidth() / 2,
                this->getY() + this->getHeight() / 2);
    }

    void print() const {
        std::cout << "Center: [" << m_x << ", " << m_y << "] ";
        std::cout << "Size: " << m_width << "x" << m_height<< " ";
        std::cout << "Score: " << m_score << std::endl;
    }

    float m_x, m_y, m_width, m_height;
    float m_score;
    int m_level;
private:
    int m_weaks;
    std::string m_filename;
    cv::Scalar m_color;
    std::string m_modelName;

    std::string detectorname;
};

bool compareByScore(const Detection *a, const Detection *b);
void SortDetections(std::vector<Detection*> &Dets);
