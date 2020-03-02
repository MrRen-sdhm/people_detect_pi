
#ifndef NONMAXIMUMSUPPRESSION_H_
#define NONMAXIMUMSUPPRESSION_H_

#include "DetectionList.h"
#include <vector>

class NonMaximumSuppression {

public:
    NonMaximumSuppression();
    virtual ~NonMaximumSuppression();

    DetectionList standardNMS(const DetectionList &DL);
    DetectionList standardNMS(const DetectionList &DL, float overlap);
    static DetectionList dollarNMS(DetectionList &DL);
};

#endif /* NONMAXIMUMSUPPRESSION_H_ */
