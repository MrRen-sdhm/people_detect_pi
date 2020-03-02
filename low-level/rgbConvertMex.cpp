/*******************************************************************************
 * Piotr's Image&Video Toolbox      Version 3.22
 * Copyright 2013 Piotr Dollar.  [pdollar-at-caltech.edu]
 * Please email me if you find bugs, or have suggestions or questions!
 * Licensed under the Simplified BSD License [see external/bsd.txt]
 *******************************************************************************/

#include <cmath>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <typeinfo>

#include "sse.hpp"

// Constants for rgb2luv conversion and lookup table for y-> l conversion
float* rgb2luv_setup(float z, float *mr, float *mg, float *mb, float &minu,
        float &minv, float &un, float &vn) {
    // set constants for conversion
    const float y0 = (float) ((6.0 / 29) * (6.0 / 29) * (6.0 / 29));
    const float a = (float) ((29.0 / 3) * (29.0 / 3) * (29.0 / 3));
    un = (float) 0.197833;
    vn = (float) 0.468331;
    mr[0] = (float) 0.430574 * z;
    mr[1] = (float) 0.222015 * z;
    mr[2] = (float) 0.020183 * z;
    mg[0] = (float) 0.341550 * z;
    mg[1] = (float) 0.706655 * z;
    mg[2] = (float) 0.129553 * z;
    mb[0] = (float) 0.178325 * z;
    mb[1] = (float) 0.071330 * z;
    mb[2] = (float) 0.939180 * z;
    float maxi = (float) 1.0 / 270;
    minu = -88 * maxi;
    minv = -134 * maxi;
    // build (padded) lookup table for y->l conversion assuming y in [0,1]
    static float lTable[1064];
    static bool lInit = false;
    if (lInit)
        return lTable;
    float y, l;
    for (int i = 0; i < 1025; i++) {
        y = (float) (i / 1024.0);
        l = y > y0 ? 116 * (float) pow((double) y, 1.0 / 3.0) - 16 : y * a;
        lTable[i] = l * maxi;
    }
    for (int i = 1025; i < 1064; i++)
        lTable[i] = lTable[i - 1];
    lInit = true;
    return lTable;
}

// Convert from rgb to luv using sse
void rgb2luv_sse(uint8_t *I, float *J, int n, float nrm) {
    const int k = 256;
    float *R = (float *) aligned_alloc(16, k * sizeof(float));
    float *G = (float *) aligned_alloc(16, k * sizeof(float));
    float *B = (float *) aligned_alloc(16, k * sizeof(float));
    assert(R);
    assert(G);
    assert(B);
    assert(((size_t )I & 15) == 0);
    assert(((size_t )J & 15) == 0);
    assert(n % 4 == 0);
    int i = 0, i1, n1;
    float minu, minv, un, vn, mr[3], mg[3], mb[3];
    float *lTable = rgb2luv_setup(nrm, mr, mg, mb, minu, minv, un, vn);
    while (i < n) {
        n1 = i + k;
        if (n1 > n)
            n1 = n;
        float *J1 = J + i;
        float *R1, *G1, *B1;
        // convert to floats (and load input into cache)
        R1 = R;
        G1 = G;
        B1 = B;
        unsigned char *Ri = I + i, *Gi = Ri + n, *Bi = Gi + n;
        for (i1 = 0; i1 < (n1 - i); i1++) {
            R1[i1] = (float) *Ri++;
            G1[i1] = (float) *Gi++;
            B1[i1] = (float) *Bi++;
        }
        // compute RGB -> XYZ
        for (int j = 0; j < 3; j++) {
            __m128 _mr, _mg, _mb, *_J = (__m128 *) (J1 + j * n);
            __m128 *_R = (__m128 *) R1, *_G = (__m128 *) G1,
                    *_B = (__m128 *) B1;
            _mr = SET(mr[j]);
            _mg = SET(mg[j]);
            _mb = SET(mb[j]);
            for (i1 = i; i1 < n1; i1 += 4)
                *(_J++) = ADD(ADD(MUL(*(_R++), _mr), MUL(*(_G++), _mg)),
                        MUL(*(_B++), _mb));
        }
        { // compute XZY -> LUV (without doing L lookup/normalization)
            __m128 _c15, _c3, _cEps, _c52, _c117, _c1024, _cun, _cvn;
            _c15 = SET(15.0f);
            _c3 = SET(3.0f);
            _cEps = SET(1e-35f);
            _c52 = SET(52.0f);
            _c117 = SET(117.0f), _c1024 = SET(1024.0f);
            _cun = SET(13 * un);
            _cvn = SET(13 * vn);
            __m128 *_X, *_Y, *_Z, _x, _y, _z;
            _X = (__m128 *) J1;
            _Y = (__m128 *) (J1 + n);
            _Z = (__m128 *) (J1 + 2 * n);
            for (i1 = i; i1 < n1; i1 += 4) {
                _x = *_X;
                _y = *_Y;
                _z = *_Z;
                _z = RCP(ADD(_x, ADD(_cEps, ADD(MUL(_c15, _y), MUL(_c3, _z)))));
                *(_X++) = MUL(_c1024, _y);
                *(_Y++) = SUB(MUL(MUL(_c52, _x), _z), _cun);
                *(_Z++) = SUB(MUL(MUL(_c117, _y), _z), _cvn);
            }
        }
        { // perform lookup for L and finalize computation of U and V
            for (i1 = i; i1 < n1; i1++)
                J[i1] = lTable[(int) J[i1]];
            __m128 *_L, *_U, *_V, _l, _cminu, _cminv;
            _L = (__m128 *) J1;
            _U = (__m128 *) (J1 + n);
            _V = (__m128 *) (J1 + 2 * n);
            _cminu = SET(minu);
            _cminv = SET(minv);
            for (i1 = i; i1 < n1; i1 += 4) {
                _l = *(_L++);
                *(_U) = SUB(MUL(_l, *_U), _cminu);
                _U++;
                *(_V) = SUB(MUL(_l, *_V), _cminv);
                _V++;
            }
        }
        i = n1;
    }
    free(R);
    free(G);
    free(B);
}
