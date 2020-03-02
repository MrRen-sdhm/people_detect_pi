/*
 * Channel.h
 */

#ifndef CHANNEL_H_
#define CHANNEL_H_

#include <iostream>



class Channel {
public:
    Channel();
    virtual ~Channel();
    void PrintChanneldata() const;
    void PrintChanneldata(int w, int h) const;
    int getWidth() const;
    int getHeight() const;
    int getnChns() const;
    float *getData() const;

protected:
    void setChanneldata(float* data, int width, int height, int nChns);

    int width;
    int height;
    int nChns;
    float *data;
};

#endif /* CHANNEL_H_ */
