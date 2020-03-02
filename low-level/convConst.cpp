/*******************************************************************************
 * Piotr's Image&Video Toolbox      Version 3.24
 * Copyright 2013 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
 * Please email me if you find bugs, or have suggestions or questions!
 * Licensed under the Simplified BSD License [see external/bsd.txt]
 *******************************************************************************/
#include "tbb/tbb.h"
#include <string.h>
#include "sse.hpp"

// convolve one column of I by a 2rx1 triangle filter
void convTriY(float *I, float *O, int h, int r, int s) {
    r++;
    float t, u;
    int j, r0 = r - 1, r1 = r + 1, r2 = 2 * h - r, h0 = r + 1, h1 = h - r + 1,
            h2 = h;
    u = t = I[0];
    for (j = 1; j < r; j++)
        u += t += I[j];
    u = 2 * u - t;
    t = 0;
    if (s == 1) {
        O[0] = u;
        j = 1;
        for (; j < h0; j++)
            O[j] = u += t += I[r - j] + I[r0 + j] - 2 * I[j - 1];
        for (; j < h1; j++)
            O[j] = u += t += I[j - r1] + I[r0 + j] - 2 * I[j - 1];
        for (; j < h2; j++)
            O[j] = u += t += I[j - r1] + I[r2 - j] - 2 * I[j - 1];
    } else {
        int k = (s - 1) / 2;
        h2 = (h / s) * s;
        if (h0 > h2)
            h0 = h2;
        if (h1 > h2)
            h1 = h2;
        if (++k == s) {
            k = 0;
            *O++ = u;
        }
        j = 1;
        for (; j < h0; j++) {
            u += t += I[r - j] + I[r0 + j] - 2 * I[j - 1];
            if (++k == s) {
                k = 0;
                *O++ = u;
            }
        }
        for (; j < h1; j++) {
            u += t += I[j - r1] + I[r0 + j] - 2 * I[j - 1];
            if (++k == s) {
                k = 0;
                *O++ = u;
            }
        }
        for (; j < h2; j++) {
            u += t += I[j - r1] + I[r2 - j] - 2 * I[j - 1];
            if (++k == s) {
                k = 0;
                *O++ = u;
            }
        }
    }
}

// convolve I by a 2rx1 triangle filter (uses SSE)
void convTri(float *I, float *O, int h, int w, int d, int r, int s) {
    r++;
    float nrm = 1.0f / (r * r * r * r);
    int i, j, k = (s - 1) / 2, h0, h1, w0;
    if (h % 4 == 0)
        h0 = h1 = h;
    else {
        h0 = h - (h % 4);
        h1 = h0 + 4;
    }
    w0 = (w / s) * s;
    float *T = (float*) aligned_alloc(16, 2 * h1 * sizeof(float)), *U = T + h1;
    while (d-- > 0) {
        // initialize T and U
        for (j = 0; j < h0; j += 4)
            STR(U[j], STR(T[j], LDu(I[j])));
        for (i = 1; i < r; i++)
            for (j = 0; j < h0; j += 4)
                INC(U[j], INC(T[j], LDu(I[j + i * h])));
        for (j = 0; j < h0; j += 4)
            STR(U[j], MUL(nrm, (SUB(MUL(2, LD(U[j])), LD(T[j])))));
        for (j = 0; j < h0; j += 4)
            STR(T[j], 0);
        for (j = h0; j < h; j++)
            U[j] = T[j] = I[j];
        for (i = 1; i < r; i++)
            for (j = h0; j < h; j++)
                U[j] += T[j] += I[j + i * h];
        for (j = h0; j < h; j++) {
            U[j] = nrm * (2 * U[j] - T[j]);
            T[j] = 0;
        }
        // prepare and convolve each column in turn
        k++;
        if (k == s) {
            k = 0;
            convTriY(U, O, h, r - 1, s);
            O += h / s;
        }
        for (i = 1; i < w0; i++) {
            float *Il = I + (i - 1 - r) * h;
            if (i <= r)
                Il = I + (r - i) * h;
            float *Im = I + (i - 1) * h;
            float *Ir = I + (i - 1 + r) * h;
            if (i > w - r)
                Ir = I + (2 * w - r - i) * h;
            for (j = 0; j < h0; j += 4) {
                INC(T[j], ADD(LDu(Il[j]), LDu(Ir[j]), MUL(-2, LDu(Im[j]))));
                INC(U[j], MUL(nrm, LD(T[j])));
            }
            for (j = h0; j < h; j++)
                U[j] += nrm * (T[j] += Il[j] + Ir[j] - 2 * Im[j]);
            k++;
            if (k == s) {
                k = 0;
                convTriY(U, O, h, r - 1, s);
                O += h / s;
            }
        }
        I += w * h;
    }
    free(T);
}

// convolve one column of I by a [1 p 1] filter (uses SSE)
void convTri1Y(float *I, float *O, int h, float p, int s) {
#define C4(m,o) ADD(ADD(LDu(I[m*j-1+o]),MUL(p,LDu(I[m*j+o]))),LDu(I[m*j+1+o]))
    int j = 0, k = ((~((size_t) O) + 1) & 15) / 4, h2 = (h - 1) / 2;
    if (s == 2) {
        for (; j < k; j++)
            O[j] = I[2 * j] + p * I[2 * j + 1] + I[2 * j + 2];
        for (; j < h2 - 4; j += 4)
            STR(O[j], _mm_shuffle_ps(C4(2, 1), C4(2, 5), 136));
        for (; j < h2; j++)
            O[j] = I[2 * j] + p * I[2 * j + 1] + I[2 * j + 2];
        if (h % 2 == 0)
            O[j] = I[2 * j] + (1 + p) * I[2 * j + 1];
    } else {
        O[j] = (1 + p) * I[j] + I[j + 1];
        j++;
        if (k == 0)
            k = (h <= 4) ? h - 1 : 4;
        for (; j < k; j++)
            O[j] = I[j - 1] + p * I[j] + I[j + 1];
        for (; j < h - 4; j += 4)
            STR(O[j], C4(1, 0));
        for (; j < h - 1; j++)
            O[j] = I[j - 1] + p * I[j] + I[j + 1];
        O[j] = I[j - 1] + (1 + p) * I[j];
    }
#undef C4
}

// convolve I by a [1 p 1] filter (uses SSE)
void convTri1(float *I, float *O, int h, int w, int d, float p, int s) {
    const float nrm = 1.0f / ((p + 2) * (p + 2));
    int i, j, h0 = h - (h % 4);
    float *Il, *Im, *Ir, *T = (float*) aligned_alloc(16, h * sizeof(float));
    for (int d0 = 0; d0 < d; d0++) {
        for (i = s / 2; i < w; i += s) {
            Il = Im = Ir = I + i * h + d0 * h * w;
            if (i > 0)
                Il -= h;
            if (i < w - 1)
                Ir += h;
            for (j = 0; j < h0; j += 4)
                STR(T[j],
                        MUL(nrm,
                                ADD(ADD(LDu(Il[j]), MUL(p, LDu(Im[j]))),
                                        LDu(Ir[j]))));
            for (j = h0; j < h; j++)
                T[j] = nrm * (Il[j] + p * Im[j] + Ir[j]);
            convTri1Y(T, O, h, p, s);
            O += h / s;
        }
    }
    free(T);
}
