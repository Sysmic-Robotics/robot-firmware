#include "lowpass_iir.h"

void LowPassFilter_Init(LowPassFilter_t *filt, float alpha)
{
    filt->y = 0.0f;
    filt->alpha = alpha;
    filt->initialized = 0;
}

void LowPassFilter_InitValue(LowPassFilter_t *filt, float alpha, float initial)
{
    filt->y = initial;
    filt->alpha = alpha;
    filt->initialized = 1;
}

float LowPassFilter_Update(LowPassFilter_t *filt, float x)
{
    if (!filt->initialized) {
        filt->y = x;
        filt->initialized = 1;
        return filt->y;
    }
    filt->y += filt->alpha * (x - filt->y);
    return filt->y;
}
