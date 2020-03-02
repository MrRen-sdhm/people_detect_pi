/*
 * GradMagChannel.h
 */

#ifndef GRADMAGCHANNEL_H_
#define GRADMAGCHANNEL_H_

#include "Channel.h"
#include "ColorChannel.h"


class GradMagChannel: public Channel {
public:
    GradMagChannel(const ColorChannel &color_channel);
    virtual ~GradMagChannel();

    const float * getMagnitude() const {
        return this->getData();
    }

    const float * getOrientation() const {
        return this->orientation;
    }

private:
    const float *orientation;
};

#endif /* GRADMAGCHANNEL_H_ */
