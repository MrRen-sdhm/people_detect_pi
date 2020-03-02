#ifndef _H_FUNC
#define _H_FUNC

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

// Some general functions which are not yet added to the separate classes

void convTri(float *I, float *O, int h, int w, int d, int r, int s);
void convTri1(float *I, float *O, int h, int w, int d, float p, int s);

void grad2(float *I, float *Gx, float *Gy, int h, int w, int d);
void gradMag(float *I, float *M, float *O, int h, int w, int d, bool full);
void gradHist(const float *magnitude, const float *orientation,
        float *histogram, int src_height, int src_width, int block_size,
        int nOrients, bool full_2pi);
void gradMagNorm(float *M, float *S, int h, int w, float norm);

void rgb2luv_sse(unsigned char *I, float *J, int n, float nrm);
#endif
