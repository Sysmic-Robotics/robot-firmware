#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdint.h>

typedef struct {
    float y;
    float alpha;
    uint8_t initialized;
} LowPassFilter_t;

void LowPassFilter_Init(LowPassFilter_t *filt, float alpha);
void LowPassFilter_InitValue(LowPassFilter_t *filt, float alpha, float initial);
float LowPassFilter_Update(LowPassFilter_t *filt, float x);

#endif
