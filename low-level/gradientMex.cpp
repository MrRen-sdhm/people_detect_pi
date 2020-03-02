/*******************************************************************************
 * Piotr's Image&Video Toolbox      Version 3.24
 * Copyright 2013 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
 * Please email me if you find bugs, or have suggestions or questions!
 * Licensed under the Simplified BSD License [see external/bsd.txt]
 *******************************************************************************/
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <cassert>
#include <stdexcept>

#include "sse.hpp"

#define PI 3.14159265f

// compute x and y gradients for just one column (uses sse)
void grad1(float *I, float *Gx, float *Gy, int h, int w, int x) {
    int y, y1;
    float *Ip, *In, r;
    __m128 *_Ip, *_In, *_G, _r;
    // compute column of Gx
    Ip = I - h;
    In = I + h;
    r = .5f;
    if (x == 0) {
        r = 1;
        Ip += h;
    } else if (x == w - 1) {
        r = 1;
        In -= h;
    }
    if (h < 4 || h % 4 > 0 || (size_t(I) & 15) || (size_t(Gx) & 15)) {
        for (y = 0; y < h; y++)
            *Gx++ = (*In++ - *Ip++) * r;
    } else {
        _G = (__m128 *) Gx;
        _Ip = (__m128 *) Ip;
        _In = (__m128 *) In;
        _r = SET(r);
        for (y = 0; y < h; y += 4)
            *_G++ = MUL(SUB(*_In++, *_Ip++), _r);
    }
    // compute column of Gy
#define GRADY(r) *Gy++=(*In++-*Ip++)*r;
    Ip = I;
    In = Ip + 1;
    // GRADY(1); Ip--; for(y=1; y<h-1; y++) GRADY(.5f); In--; GRADY(1);
    y1 = ((~((size_t) Gy) + 1) & 15) / 4;
    if (y1 == 0)
        y1 = 4;
    if (y1 > h - 1)
        y1 = h - 1;
    GRADY(1);
    Ip--;
    for (y = 1; y < y1; y++)
        GRADY(.5f);
    _r = SET(.5f);
    _G = (__m128 *) Gy;
    for (; y + 4 < h - 1; y += 4, Ip += 4, In += 4, Gy += 4)
        *_G++ = MUL(SUB(LDu(*In), LDu(*Ip)), _r);
    for (; y < h - 1; y++)
        GRADY(.5f);
    In--;
    GRADY(1);
#undef GRADY
}

// compute x and y gradients at each location (uses sse)
void grad2(float *I, float *Gx, float *Gy, int h, int w, int d) {
    int o, x, c, a = w * h;
    for (c = 0; c < d; c++)
        for (x = 0; x < w; x++) {
            o = c * a + x * h;
            grad1(I + o, Gx + o, Gy + o, h, w, x);
        }
}

// build lookup table a[] s.t. a[x*n]~=acos(x) for x in [-1,1]
float* acosTable() {
    const int n = 10000, b = 10;
    int i;
    static float a[n * 2 + b * 2];
    static bool init = false;
    float *a1 = a + n + b;
    if (init)
        return a1;
    for (i = -n - b; i < -n; i++)
        a1[i] = PI;
    for (i = -n; i < n; i++)
        a1[i] = float(acos(i / float(n)));
    for (i = n; i < n + b; i++)
        a1[i] = 0;
    for (i = -n - b; i < n / 10; i++)
        if (a1[i] > PI - 1e-6f)
            a1[i] = PI - 1e-6f;
    init = true;
    return a1;
}

// compute gradient magnitude and orientation at each location (uses sse)
void gradMag(float *I, float *M, float *O, int h, int w, int d, bool full) {
    int x, y, y1, c, h4, s;
    float *Gx, *Gy, *M2;
    __m128 *_Gx, *_Gy, *_M2, _m;
    float *acost = acosTable(), acMult = 10000.0f;
    // allocate memory for storing one column of output (padded so h4%4==0)
    h4 = (h % 4 == 0) ? h : h - (h % 4) + 4;
    s = d * h4 * sizeof(float);
    M2 = (float*) aligned_alloc(16, s);
    _M2 = (__m128 *) M2;
    Gx = (float*) aligned_alloc(16, s);
    _Gx = (__m128 *) Gx;
    Gy = (float*) aligned_alloc(16, s);
    _Gy = (__m128 *) Gy;
    // compute gradient magnitude and orientation for each column
    for (x = 0; x < w; x++) {
        // compute gradients (Gx, Gy) with maximum squared magnitude (M2)
        for (c = 0; c < d; c++) {
            grad1(I + x * h + c * w * h, Gx + c * h4, Gy + c * h4, h, w, x);
            for (y = 0; y < h4 / 4; y++) {
                y1 = h4 / 4 * c + y;
                _M2[y1] = ADD(MUL(_Gx[y1], _Gx[y1]), MUL(_Gy[y1], _Gy[y1]));
                if (c == 0)
                    continue;
                _m = CMPGT(_M2[y1], _M2[y]);
                _M2[y] = OR(AND(_m, _M2[y1]), ANDNOT(_m, _M2[y]));
                _Gx[y] = OR(AND(_m, _Gx[y1]), ANDNOT(_m, _Gx[y]));
                _Gy[y] = OR(AND(_m, _Gy[y1]), ANDNOT(_m, _Gy[y]));
            }
        }
        // compute gradient mangitude (M) and normalize Gx
        for (y = 0; y < h4 / 4; y++) {
            _m = MIN_(RCPSQRT(_M2[y]), SET(1e10f));
            _M2[y] = RCP(_m);
            if (O)
                _Gx[y] = MUL(MUL(_Gx[y], _m), SET(acMult));
            if (O)
                _Gx[y] = XOR(_Gx[y], AND(_Gy[y], SET(-0.f)));
        };
        memcpy(M + x * h, M2, h * sizeof(float));
        // compute and store gradient orientation (O) via table lookup
        if (O != 0)
            for (y = 0; y < h; y++)
                O[x * h + y] = acost[(int) Gx[y]];
        if (O != 0 && full) {
            y1 = ((~size_t(O + x * h) + 1) & 15) / 4;
            y = 0;
            for (; y < y1; y++)
                O[y + x * h] += (Gy[y] < 0) * PI;
            for (; y < h - 4; y += 4)
                STRu(O[y + x * h],
                        ADD(LDu(O[y + x * h]),
                                AND(CMPLT(LDu(Gy[y]), SET(0.f)), SET(PI))));
            for (; y < h; y++)
                O[y + x * h] += (Gy[y] < 0) * PI;
        }
    }
    free(Gx);
    free(Gy);
    free(M2);
}

// normalize gradient magnitude at each location (uses sse)
void gradMagNorm(float *M, float *S, int h, int w, float norm) {
    __m128 *_M, *_S, _norm;
    int i = 0, n = h * w, n4 = n / 4;
    _S = (__m128 *) S;
    _M = (__m128 *) M;
    _norm = SET(norm);
    bool sse = !(size_t(M) & 15) && !(size_t(S) & 15);
    if (sse)
        for (; i < n4; i++) {
            *_M = MUL(*_M, RCP(ADD(*_S++, _norm)));
            _M++;
        }
    if (sse)
        i *= 4;
    for (; i < n; i++)
        M[i] /= (S[i] + norm);
}

// helper for gradHist, quantize O and M into O0, O1 and M0, M1 (uses sse)
void gradQuantize(const float *orientation_column,
        const float *magnitude_column, int *O0, int *O1, float *M0, float *M1,
        int n_blocks, int n, float norm, int n_orients, bool full_2pi) {
    // assumes all *OUTPUT* matrices are 4-byte aligned
    int i, o0, o1;
    float o, od, m;
    __m128i *_O0, *_O1;
    __m128 *_M0, *_M1;
    // define useful constants
    const float rad_to_orient = (float) n_orients / (full_2pi ? 2 * PI : PI);
    const int oMax = n_orients * n_blocks;
    // perform the majority of the work with sse
    _O0 = (__m128i *) O0;
    _O1 = (__m128i *) O1;
    _M0 = (__m128 *) M0;
    _M1 = (__m128 *) M1;
    for (i = 0; i <= n - 4; i += 4) {
        // 弧度 -> 方向编号
        __m128 _o = MUL(LDu(orientation_column[i]), SET(rad_to_orient));
        // 限制方向编号的范围
        _o = _mm_max_ps(_o, SET(0.0f));
        _o = _mm_min_ps(_o, SET(n_orients - 0.001f));
        // 计算内插
        __m128i _o0 = CVT(_o);          // 取整
        __m128 _od = SUB(_o, CVT(_o0)); // 后一个方向的比重
        _o0 = CVT(MUL(CVT(_o0), SET((float) n_blocks)));    // 计算前一角度对应的通道起始索引
        // 限制范围
//        _o0 = AND(CMPGT(SET(oMax), _o0), _o0);
//        if (_mm_movemask_epi8(_mm_cmplt_epi32(_o0, _mm_set1_epi32(0)))) {
//            throw std::runtime_error("_o0 should >= 0");
//        }
        *_O0 = _o0;

        __m128i _o1 = ADD(_o0, SET(n_blocks));  // 计算后一角度对应的通道起始索引
        // 限制范围, 处理5+1=6的情况
        _o1 = AND(CMPGT(SET(oMax), _o1), _o1);
        *_O1 = _o1;

        __m128 _m = MUL(LDu(magnitude_column[i]), SET(norm));   // 幅值系数
        *_M1 = MUL(_od, _m);    // 后一个角度的幅值
        *_M0 = SUB(_m, *_M1);   // 前一个角度的幅值

        _O0++;
        _O1++;
        _M0++;
        _M1++;
    }
    // compute trailing locations without sse
    for (i; i < n; i++) {
        float o = orientation_column[i] * rad_to_orient;
        o = std::max(o, 0.0f);
        o = std::min(o, n_orients - 0.001f);
        int o0 = (int) o;
        float od = o - o0;
        o0 *= n_blocks;
//        o0 %= oMax;
//        if (o0 < 0) {
//            throw std::runtime_error("o0 should >= 0");
//        }
        O0[i] = o0;
        o1 = o0 + n_blocks;
        o1 %= oMax;
        O1[i] = o1;
        m = magnitude_column[i] * norm;
        M1[i] = od * m;
        M0[i] = m - M1[i];
    }
}

// compute nOrients gradient histograms per bin x bin block of pixels
void gradHist(const float *magnitude, const float *orientation,
        float *histogram, int src_height, int src_width, int block_size,
        int n_orients, bool full_2pi) {
    const int height_block = src_height / block_size;
    const int width_block = src_width / block_size;
    const int h0 = height_block * block_size;
    const int w0 = width_block * block_size;
    const int n_blocks = width_block * height_block;

    int* O0 = (int*) aligned_alloc(16, src_height * sizeof(int));
    if (O0 == NULL) {
        throw std::runtime_error("Failed to aligned_alloc O0");
    }
    float* M0 = (float*) aligned_alloc(16, src_height * sizeof(float));
    if (M0 == NULL) {
        throw std::runtime_error("Failed to aligned_alloc M0");
    }
    int* O1 = (int*) aligned_alloc(16, src_height * sizeof(int));
    if (O1 == NULL) {
        throw std::runtime_error("Failed to aligned_alloc O1");
    }
    float* M1 = (float*) aligned_alloc(16, src_height * sizeof(float));
    if (M1 == NULL) {
        throw std::runtime_error("Failed to aligned_alloc M1");
    }

    // main loop
    for (int x = 0; x < w0; x++) {
        // compute target orientation bins for entire column - very fast
        gradQuantize(orientation + x * src_height, magnitude + x * src_height,
                O0, O1, M0, M1, n_blocks, h0, 1.0f / block_size / block_size,
                n_orients, full_2pi);

        // interpolate w.r.t. orientation only, not spatial bin
        float* hist_col = histogram + (x / block_size) * height_block;
        //    if (&H1[O0[y]] - H >= h * w * nOrients) {\
            //        throw std::runtime_error("H1[O0[y]] overflow");\
            //    }\
            //    if (&H1[O1[y]] - H >= h * w * nOrients) {\
            //        throw std::runtime_error("H1[O1[y]] overflow");\
            //    }
        for (int y = 0; y < h0;) {
            for (int i = 0; i < block_size; i++) {
                hist_col[O0[y]] += M0[y];
                hist_col[O1[y]] += M1[y];
                y++;
            }
            hist_col++;
        }
    }
    free(O0);
    free(O1);
    free(M0);
    free(M1);
}

