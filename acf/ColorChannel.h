/*
 * ColorChannel.h
 */

#ifndef COLORCHANNEL_H_
#define COLORCHANNEL_H_

#include <string>
#include "Channel.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"

class ColorChannel : public Channel {
public:

    ColorChannel(float* image_yuv, size_t image_width, size_t image_height);
    virtual ~ColorChannel();
};

#endif /* COLORCHANNEL_H_ */
