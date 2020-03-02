/*
 * GradHistChannel.h
 */

#ifndef GRADHISTCHANNEL_H_
#define GRADHISTCHANNEL_H_

#include "Channel.h"
#include "GradMagChannel.h"

class GradHistChannel: public Channel {
public:
    GradHistChannel(const GradMagChannel &grad_mag_channel, uint32_t shrink);
    virtual ~GradHistChannel();

};

#endif /* GRADHISTCHANNEL_H_ */
